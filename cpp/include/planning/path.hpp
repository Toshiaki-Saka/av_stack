#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ad::planning {

// Cubic-Hermite smooth step from 0→1 over [a, b].
static inline double smoothstep(double x, double a, double b) {
    double t = std::max(0.0, std::min(1.0, (x - a) / (b - a)));
    return t * t * (3.0 - 2.0 * t);
}

// Static double-lane-change path.  Interface mirrors Trajectory so callers
// can swap Path ↔ Trajectory without changes.
struct Path {
    std::vector<double> x, y, psi, kappa, v;
    int    n     = 0;
    double dt    = 0.1;
    double v_ref = 12.0;
    double L     = 2.7;

    Path() = default;

    explicit Path(double v_ref_ = 12.0, double dt_ = 0.1, double T = 14.0,
                  double L_ = 2.7, double lane = 3.5)
        : dt(dt_), v_ref(v_ref_), L(L_)
    {
        n = static_cast<int>(T / dt_) + 1;
        x.resize(n); y.resize(n); v.assign(n, v_ref_);
        psi.resize(n); kappa.resize(n, 0.0);

        for (int i = 0; i < n; ++i) {
            x[i] = v_ref_ * (i * dt_);
            y[i] = lane * (smoothstep(x[i], 25.0, 45.0) -
                           smoothstep(x[i], 80.0, 100.0));
        }

        // Finite-difference derivatives for psi and kappa
        auto grad1 = [&](const std::vector<double>& f, std::vector<double>& out) {
            out.resize(n);
            out[0]   = f[1]   - f[0];
            out[n-1] = f[n-1] - f[n-2];
            for (int i = 1; i < n-1; ++i)
                out[i] = (f[i+1] - f[i-1]) * 0.5;
        };

        std::vector<double> dx, dy, ddx, ddy;
        grad1(x, dx); grad1(y, dy);
        grad1(dx, ddx); grad1(dy, ddy);

        for (int i = 0; i < n; ++i) {
            psi[i] = std::atan2(dy[i], dx[i]);
            double denom = dx[i]*dx[i] + dy[i]*dy[i];
            kappa[i] = (denom > 1e-12)
                ? (dx[i]*ddy[i] - dy[i]*ddx[i]) / std::pow(denom, 1.5)
                : 0.0;
        }
    }

    int nearest(double px, double py) const {
        int    best    = 0;
        double best_d2 = 1e18;
        for (int i = 0; i < n; ++i) {
            double d2 = (x[i]-px)*(x[i]-px) + (y[i]-py)*(y[i]-py);
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        return best;
    }

    std::array<double,4> errors(double sx, double sy, double spsi, double /*sv*/) const {
        int    i    = nearest(sx, sy);
        double ppsi = psi[i];
        double dx_  = sx - x[i], dy_ = sy - y[i];
        double e_y  = -std::sin(ppsi)*dx_ + std::cos(ppsi)*dy_;
        double de   = spsi - ppsi;
        double e_psi = std::atan2(std::sin(de), std::cos(de));
        return {e_y, e_psi, kappa[i], v[i]};
    }

    // Steering feedforward from curvature (matches path.py, unlike trajectory.py).
    void mpc_window(double sx, double sy, int N,
                    std::vector<double>& sref,
                    std::vector<double>& uref) const
    {
        int i0 = nearest(sx, sy);
        sref.resize((N+1)*4);
        uref.resize( N   *2);

        for (int k = 0; k <= N; ++k) {
            int idx = std::min(i0 + k, n - 1);
            sref[k*4+0] = x[idx];
            sref[k*4+1] = y[idx];
            sref[k*4+2] = psi[idx];
            sref[k*4+3] = v[idx];
        }
        for (int k = 0; k < N; ++k) {
            int j     = std::min(i0 + k,     n - 1);
            int jnext = std::min(i0 + k + 1, n - 1);
            uref[k*2+0] = (v[jnext] - v[j]) / dt;
            uref[k*2+1] = std::atan(L * kappa[j]);
        }
    }
};

}  // namespace ad::planning
