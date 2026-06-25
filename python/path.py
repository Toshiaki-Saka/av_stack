"""Reference trajectory for the controller comparison.

A straight road with a double lane-change in lateral position, parameterised by
time assuming nominal forward speed v_ref. Provides:
  * sampled state arrays [x, y, psi, v] at the control timestep,
  * curvature kappa(t),
  * a nearest-point query returning (e_y signed cross-track, e_psi heading error,
    kappa, v_ref) for the PID / LQR controllers,
  * a window (sref, uref) for the MPC.
"""
import numpy as np


def _smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


class Path:
    def __init__(self, v_ref=12.0, dt=0.1, T=14.0, L=2.7, lane=3.5):
        self.dt, self.v_ref, self.L = dt, v_ref, L
        n = int(T / dt) + 1
        t = np.arange(n) * dt
        x = v_ref * t                      # forward progress (lateral motion is small)
        # double lane change: out to `lane`, then back
        y = lane * (_smoothstep(x, 25, 45) - _smoothstep(x, 80, 100))
        dx = np.gradient(x); dy = np.gradient(y)
        psi = np.arctan2(dy, dx)
        ddy = np.gradient(dy); ddx = np.gradient(dx)
        kappa = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5 + 1e-9)
        self.x, self.y, self.psi, self.kappa = x, y, psi, kappa
        self.v = np.full(n, v_ref)
        self.n = n

    def nearest(self, x, y):
        d2 = (self.x - x) ** 2 + (self.y - y) ** 2
        i = int(np.argmin(d2))
        return i

    def errors(self, state):
        """Signed cross-track e_y, heading error e_psi, local kappa, v_ref."""
        x, y, psi, v = state
        i = self.nearest(x, y)
        px, py, ppsi = self.x[i], self.y[i], self.psi[i]
        # signed lateral offset: left of path tangent is positive
        dxv, dyv = x - px, y - py
        e_y = -np.sin(ppsi) * dxv + np.cos(ppsi) * dyv
        e_psi = np.arctan2(np.sin(psi - ppsi), np.cos(psi - ppsi))
        return e_y, e_psi, float(self.kappa[i]), float(self.v[i])

    def mpc_window(self, state, N):
        """(sref:(N+1)*4, uref:N*2) starting near the car, with feedforward inputs."""
        i = self.nearest(state[0], state[1])
        idx = np.minimum(i + np.arange(N + 1), self.n - 1)
        sref = np.zeros((N + 1, 4))
        sref[:, 0] = self.x[idx]; sref[:, 1] = self.y[idx]
        sref[:, 2] = self.psi[idx]; sref[:, 3] = self.v[idx]
        uref = np.zeros((N, 2))
        for k in range(N):
            j = idx[k]
            uref[k, 0] = (self.v[idx[k + 1]] - self.v[j]) / self.dt   # a_ff
            uref[k, 1] = np.arctan(self.L * self.kappa[j])            # delta_ff
        return sref.reshape(-1).tolist(), uref.reshape(-1).tolist()
