#pragma once
#include <cmath>
#include "control/linalg.hpp"

namespace ctrl {

// --- PID with integral clamping (anti-windup) and output saturation ---
struct PID {
    double kp = 1, ki = 0, kd = 0;
    double out_min = -1e9, out_max = 1e9, i_min = -1e9, i_max = 1e9;
    double integ = 0, prev_err = 0;
    bool has_prev = false;

    double step(double err, double dt) {
        integ += err * dt;
        if (integ > i_max) integ = i_max;
        if (integ < i_min) integ = i_min;
        double deriv = has_prev ? (err - prev_err) / dt : 0.0;
        prev_err = err; has_prev = true;
        double u = kp * err + ki * integ + kd * deriv;
        if (u > out_max) u = out_max;
        if (u < out_min) u = out_min;
        return u;
    }
    void reset() { integ = 0; prev_err = 0; has_prev = false; }
};

// --- Discrete LQR: solve the DARE by backward iteration to convergence ---
// P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA ;  K = (R + B'PB)^-1 B'PA
inline Mat dlqr(const Mat& A, const Mat& B, const Mat& Q, const Mat& R,
                int iters = 1000, double tol = 1e-10) {
    Mat P = Q;
    Mat At = T(A), Bt = T(B);
    Mat Kp;
    for (int it = 0; it < iters; ++it) {
        Mat S = R + Bt * P * B;          // m×m
        Mat K = solve(S, Bt * P * A);    // m×n  = S^-1 B'PA
        Mat Pn = Q + At * P * A - At * P * B * K;
        double diff = 0;
        for (size_t i = 0; i < Pn.d.size(); ++i) diff += std::fabs(Pn.d[i] - P.d[i]);
        P = Pn;
        if (diff < tol) break;
    }
    Mat S = R + Bt * P * B;
    return solve(S, Bt * P * A);          // gain K (m×n)
}

// Lateral path-tracking LQR on the 2-state error model [e_y, e_psi] at speed v:
//   e_y'   = v * e_psi
//   e_psi' = (v/L) * delta - v * kappa
// Discretised with dt; feedforward delta_ff = atan(L*kappa) cancels the curvature.
struct LateralLQR {
    double L = 2.7, dt = 0.1;
    double q_ey = 1.0, q_epsi = 1.0, r_delta = 2.0;

    // returns steering command given cross-track error, heading error, speed, curvature
    double steer(double e_y, double e_psi, double v, double kappa) const {
        double vv = std::max(0.5, v);           // avoid singular model at standstill
        Mat A(2, 2), B(2, 1), Q(2, 2), R(1, 1);
        A(0, 0) = 1; A(0, 1) = vv * dt; A(1, 0) = 0; A(1, 1) = 1;
        B(0, 0) = 0; B(1, 0) = (vv / L) * dt;
        Q(0, 0) = q_ey; Q(1, 1) = q_epsi; R(0, 0) = r_delta;
        Mat K = dlqr(A, B, Q, R);
        double fb = -(K(0, 0) * e_y + K(0, 1) * e_psi);
        double ff = std::atan(L * kappa);
        return fb + ff;
    }
};

}  // namespace ctrl
