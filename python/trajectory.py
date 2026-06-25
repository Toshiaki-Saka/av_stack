"""A sampled reference trajectory shared by the planner and the controllers.

Holds time-sampled arrays [x, y, psi, v] (+ curvature kappa) at the control
timestep, and exposes the queries the controllers need:
  * errors(state)      -> (e_y signed cross-track, e_psi heading error, kappa, v_ref)
  * mpc_window(state,N)-> (sref:(N+1)*4, uref:N*2) with feedforward inputs
Both the static road `Path` and the planner build one of these, so PID / LQR / MPC
consume the planner's output exactly as they consume a fixed road.
"""
import numpy as np


class Trajectory:
    def __init__(self, x, y, psi=None, v=None, kappa=None, dt=0.1, L=2.7):
        self.x = np.asarray(x, float); self.y = np.asarray(y, float)
        self.dt, self.L = dt, L
        self.n = len(self.x)
        if psi is None or kappa is None:
            dx = np.gradient(self.x); dy = np.gradient(self.y)
            psi = np.arctan2(dy, dx) if psi is None else np.asarray(psi, float)
            ddx = np.gradient(dx); ddy = np.gradient(dy)
            denom = dx * dx + dy * dy
            safe = denom > 1e-6
            kappa = np.zeros_like(dx)
            kappa[safe] = (dx * ddy - dy * ddx)[safe] / np.power(denom[safe], 1.5)
        self.psi = np.asarray(psi, float)
        self.kappa = np.asarray(kappa, float)
        self.v = np.full(self.n, 0.0) if v is None else np.asarray(v, float)

    def nearest(self, x, y):
        return int(np.argmin((self.x - x) ** 2 + (self.y - y) ** 2))

    def errors(self, state):
        x, y, psi, v = state
        i = self.nearest(x, y)
        ppsi = self.psi[i]
        e_y = -np.sin(ppsi) * (x - self.x[i]) + np.cos(ppsi) * (y - self.y[i])
        e_psi = np.arctan2(np.sin(psi - ppsi), np.cos(psi - ppsi))
        return e_y, e_psi, float(self.kappa[i]), float(self.v[i])

    def mpc_window(self, state, N):
        i = self.nearest(state[0], state[1])
        idx = np.minimum(i + np.arange(N + 1), self.n - 1)
        sref = np.zeros((N + 1, 4))
        sref[:, 0] = self.x[idx]; sref[:, 1] = self.y[idx]
        sref[:, 2] = self.psi[idx]; sref[:, 3] = self.v[idx]
        uref = np.zeros((N, 2))
        for k in range(N):
            j = idx[k]; jnext = idx[k + 1]
            uref[k, 0] = (self.v[jnext] - self.v[j]) / self.dt
            # Feedforward delta from heading-rate: Δψ = v·tan(δ)/L·dt  →  δ = atan(L·Δψ/(v·dt))
            # This correctly encodes the required steering for curves (incl. lane-change
            # profiles where kappa-based δ is nearly zero at the ramp endpoints).
            dpsi = float(np.arctan2(np.sin(self.psi[jnext] - self.psi[j]),
                                    np.cos(self.psi[jnext] - self.psi[j])))
            uref[k, 1] = float(np.arctan(self.L * dpsi / max(self.v[j] * self.dt, 1e-3)))
        return sref.reshape(-1).tolist(), uref.reshape(-1).tolist()
