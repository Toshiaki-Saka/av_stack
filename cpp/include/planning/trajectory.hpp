#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ad::planning {

// Central-difference gradient — replicates numpy.gradient behaviour.
// One-sided first-order differences at boundaries, central second-order inside.
static inline void gradient(const std::vector<double>& f, std::vector<double>& out) {
    const int n = static_cast<int>(f.size());
    out.assign(n, 0.0);
    if (n < 2) return;
    out[0]   = f[1]   - f[0];
    out[n-1] = f[n-1] - f[n-2];
    for (int i = 1; i < n - 1; ++i)
        out[i] = (f[i+1] - f[i-1]) * 0.5;
}

struct Trajectory {
    std::vector<double> x, y, psi, v, kappa;
    int    n  = 0;
    double dt = 0.1;
    double L  = 2.7;

    Trajectory() = default;

    // Builds a trajectory from sampled (x, y) points plus optional v.
    // psi and kappa are always auto-computed from finite differences.
    Trajectory(std::vector<double> x_,
               std::vector<double> y_,
               std::vector<double> v_  = {},
               double              dt_ = 0.1,
               double              L_  = 2.7)
        : x(std::move(x_)), y(std::move(y_)), dt(dt_), L(L_)
    {
        n = static_cast<int>(x.size());

        v = v_.empty() ? std::vector<double>(n, 0.0) : std::move(v_);

        std::vector<double> dx, dy;
        gradient(x, dx);
        gradient(y, dy);

        psi.resize(n);
        for (int i = 0; i < n; ++i)
            psi[i] = std::atan2(dy[i], dx[i]);

        std::vector<double> ddx, ddy;
        gradient(dx, ddx);
        gradient(dy, ddy);

        kappa.resize(n, 0.0);
        for (int i = 0; i < n; ++i) {
            double denom = dx[i]*dx[i] + dy[i]*dy[i];
            if (denom > 1e-12)
                kappa[i] = (dx[i]*ddy[i] - dy[i]*ddx[i]) / std::pow(denom, 1.5);
        }
    }

    int nearest(double px, double py) const {
        int    best   = 0;
        double best_d2 = 1e18;
        for (int i = 0; i < n; ++i) {
            double d2 = (x[i]-px)*(x[i]-px) + (y[i]-py)*(y[i]-py);
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        return best;
    }

    // Returns {e_y, e_psi, kappa_ref, v_ref} for MPC error computation.
    std::array<double,4> errors(double sx, double sy, double spsi, double /*sv*/) const {
        int    i    = nearest(sx, sy);
        double ppsi = psi[i];
        double dx_  = sx - x[i], dy_ = sy - y[i];
        double e_y  = -std::sin(ppsi)*dx_ + std::cos(ppsi)*dy_;
        double de   = spsi - ppsi;
        double e_psi = std::atan2(std::sin(de), std::cos(de));
        return {e_y, e_psi, kappa[i], v[i]};
    }

    // Fills sref (flat (N+1)*4, row-major [x,y,psi,v]) and uref (flat N*2 [a,delta]).
    // Steering feedforward uses heading-rate / velocity (matches trajectory.py).
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
            double dpsi = psi[jnext] - psi[j];
            dpsi = std::atan2(std::sin(dpsi), std::cos(dpsi));
            uref[k*2+1] = std::atan(L * dpsi / std::max(v[j] * dt, 1e-3));
        }
    }
};

}  // namespace ad::planning
