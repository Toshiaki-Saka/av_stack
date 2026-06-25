#pragma once
// Kinematic bicycle model used by the simulator and as the MPC/LQR prediction model.
// State s = [x, y, psi, v], input u = [a, delta].
//   xdot = v cos psi,  ydot = v sin psi,  psidot = v/L tan delta,  vdot = a
#include <array>
#include <cmath>
#include "control/linalg.hpp"

namespace ctrl {

struct Bicycle {
    double L = 2.7;  // wheelbase [m]

    using State = std::array<double, 4>;
    using Input = std::array<double, 2>;

    State deriv(const State& s, const Input& u) const {
        double psi = s[2], v = s[3], a = u[0], delta = u[1];
        return {v * std::cos(psi), v * std::sin(psi), v / L * std::tan(delta), a};
    }

    // RK4 one-step integration over dt.
    State step(const State& s, const Input& u, double dt) const {
        auto add = [](const State& a, const State& b, double h) {
            return State{a[0] + h * b[0], a[1] + h * b[1], a[2] + h * b[2], a[3] + h * b[3]};
        };
        State k1 = deriv(s, u);
        State k2 = deriv(add(s, k1, dt / 2), u);
        State k3 = deriv(add(s, k2, dt / 2), u);
        State k4 = deriv(add(s, k3, dt), u);
        return {s[0] + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
                s[1] + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
                s[2] + dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]),
                s[3] + dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])};
    }

    // Discrete-time Jacobians (A = df/ds, B = df/du) of the RK4 step, by central
    // finite differences. Used to build the LTV model for MPC/LQR. Returning these
    // numerically (rather than hand-deriving the RK4 chain) keeps the linearisation
    // provably consistent with the nonlinear step that the plant actually uses.
    void linearize(const State& s0, const Input& u0, double dt, Mat& A, Mat& B) const {
        A = Mat(4, 4); B = Mat(4, 2);
        const double eps = 1e-6;
        for (int j = 0; j < 4; ++j) {
            State sp = s0, sm = s0; sp[j] += eps; sm[j] -= eps;
            State fp = step(sp, u0, dt), fm = step(sm, u0, dt);
            for (int i = 0; i < 4; ++i) A(i, j) = (fp[i] - fm[i]) / (2 * eps);
        }
        for (int j = 0; j < 2; ++j) {
            Input up = u0, um = u0; up[j] += eps; um[j] -= eps;
            State fp = step(s0, up, dt), fm = step(s0, um, dt);
            for (int i = 0; i < 4; ++i) B(i, j) = (fp[i] - fm[i]) / (2 * eps);
        }
    }
};

}  // namespace ctrl
