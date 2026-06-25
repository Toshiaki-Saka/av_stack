#pragma once
// Linear time-varying MPC for the kinematic bicycle.
//
// Around a reference trajectory (sref_k, uref_k) we linearise the RK4 step at each
// stage to A_k, B_k and an affine defect d_k = step(sref_k,uref_k) - sref_{k+1}.
// In error coordinates e_k = s_k - sref_k:   e_{k+1} = A_k e_k + B_k du_k + d_k.
// Condensing over the horizon gives  E = Sx e0 + Su dU + Sd, and a strongly convex
// box-constrained QP  min  E'Qbar E + dU'Rbar dU  s.t.  umin <= uref+dU <= umax,
// solved from scratch with accelerated projected gradient (FISTA).
#include <vector>
#include <cmath>
#include <algorithm>
#ifndef M_PI
static constexpr double M_PI = 3.14159265358979323846;
#endif
#include "control/linalg.hpp"
#include "control/bicycle.hpp"

namespace ctrl {

struct MPC {
    Bicycle model;
    int N = 15;
    double dt = 0.1;
    // diagonal weights on [x,y,psi,v] and [a,delta]
    double q_x = 1, q_y = 8, q_psi = 4, q_v = 2;
    double r_a = 1, r_delta = 4;
    double a_min = -6, a_max = 3, delta_min = -0.6, delta_max = 0.6;
    int iters = 200;

    using State = Bicycle::State;
    using Input = Bicycle::Input;

    // sref: (N+1)*4 flat, uref: N*2 flat. Returns the first optimal input.
    Input solve(const State& s0,
                const std::vector<double>& sref,
                const std::vector<double>& uref) const {
        const int n = 4, m = 2;
        // Per-stage linearisation + defect
        std::vector<Mat> A(N), B(N);
        std::vector<std::array<double, 4>> d(N);
        for (int k = 0; k < N; ++k) {
            State sk = {sref[k * n], sref[k * n + 1], sref[k * n + 2], sref[k * n + 3]};
            Input uk = {uref[k * m], uref[k * m + 1]};
            model.linearize(sk, uk, dt, A[k], B[k]);
            State f = model.step(sk, uk, dt);
            for (int i = 0; i < n; ++i)
                d[k][i] = f[i] - sref[(k + 1) * n + i];
        }
        // e0
        std::vector<double> e0(n);
        for (int i = 0; i < n; ++i) e0[i] = s0[i] - sref[i];
        // wrap heading error to [-pi,pi]
        while (e0[2] > M_PI) e0[2] -= 2 * M_PI;
        while (e0[2] < -M_PI) e0[2] += 2 * M_PI;

        // Build Su (Nn x Nm), and stacked offset off_k = Sx e0 + Sd (length Nn)
        Mat Su(N * n, N * m);
        std::vector<double> off(N * n, 0.0);
        // state transition products via forward recursion of blocks
        // We compute, for each output stage k (1..N), contributions from each input j<k.
        // Maintain Phi = product of A from j+1..k as we go.
        for (int kk = 1; kk <= N; ++kk) {
            int k = kk - 1;  // output index (e_{kk}) row block = k
            // offset: e_kk from e0 and defects
            // propagate e0 and d through A's
            std::vector<double> acc(n, 0.0);
            // start with e0 then apply A_0..A_{kk-1}; accumulate defects
            std::vector<double> state = e0;
            for (int t = 0; t < kk; ++t) {
                // state = A_t state + d_t
                std::vector<double> ns(n, 0.0);
                for (int i = 0; i < n; ++i) {
                    double s = 0;
                    for (int p = 0; p < n; ++p) s += A[t](i, p) * state[p];
                    ns[i] = s + d[t][i];
                }
                state = ns;
            }
            for (int i = 0; i < n; ++i) off[k * n + i] = state[i];

            // Su blocks: e_kk includes sum_{j=0}^{kk-1} Phi(kk, j+1) B_j du_j
            // Phi(kk, j+1) = A_{kk-1} ... A_{j+1}
            for (int j = 0; j < kk; ++j) {
                // build M = Phi(kk, j+1) * B_j  (n x m)
                Mat P = Mat::eye(n);
                for (int t = j + 1; t < kk; ++t) P = A[t] * P;
                Mat MB = P * B[j];
                for (int i = 0; i < n; ++i)
                    for (int c = 0; c < m; ++c)
                        Su(k * n + i, j * m + c) = MB(i, c);
            }
        }

        // Weights
        std::vector<double> q = {q_x, q_y, q_psi, q_v};
        std::vector<double> rr = {r_a, r_delta};
        // H = Su' Qbar Su + Rbar  ;  g = 2 Su' Qbar off
        int M = N * m;
        Mat H(M, M);
        std::vector<double> g(M, 0.0);
        // Su' Qbar Su
        for (int a = 0; a < M; ++a) {
            for (int b = 0; b < M; ++b) {
                double s = 0;
                for (int row = 0; row < N * n; ++row) {
                    double w = q[row % n];
                    s += Su(row, a) * w * Su(row, b);
                }
                H(a, b) = s;
            }
            H(a, a) += rr[a % m];
            double gg = 0;
            for (int row = 0; row < N * n; ++row)
                gg += Su(row, a) * q[row % n] * off[row];
            g[a] = 2.0 * gg;
        }

        // Box bounds on dU: umin - uref <= dU <= umax - uref
        std::vector<double> lo(M), hi(M);
        for (int k = 0; k < N; ++k) {
            lo[k * m + 0] = a_min - uref[k * m + 0];     hi[k * m + 0] = a_max - uref[k * m + 0];
            lo[k * m + 1] = delta_min - uref[k * m + 1]; hi[k * m + 1] = delta_max - uref[k * m + 1];
        }

        // Lipschitz constant of grad (2H): power iteration for largest eigenvalue
        double Lf = power_eig(H) * 2.0 + 1e-9;
        double alpha = 1.0 / Lf;

        // FISTA with box projection
        std::vector<double> x(M, 0.0), y = x, xprev = x;
        double tprev = 1.0;
        for (int it = 0; it < iters; ++it) {
            // grad at y: 2 H y + g
            std::vector<double> grad(M, 0.0);
            for (int i = 0; i < M; ++i) {
                double s = 0;
                for (int j = 0; j < M; ++j) s += H(i, j) * y[j];
                grad[i] = 2.0 * s + g[i];
            }
            std::vector<double> xnew(M);
            for (int i = 0; i < M; ++i) {
                double v = y[i] - alpha * grad[i];
                xnew[i] = std::min(hi[i], std::max(lo[i], v));  // projection onto box
            }
            double t = 0.5 * (1 + std::sqrt(1 + 4 * tprev * tprev));
            for (int i = 0; i < M; ++i) y[i] = xnew[i] + (tprev - 1) / t * (xnew[i] - x[i]);
            x = xnew; tprev = t;
        }
        Input u0 = {uref[0] + x[0], uref[1] + x[1]};
        u0[0] = std::min(a_max, std::max(a_min, u0[0]));
        u0[1] = std::min(delta_max, std::max(delta_min, u0[1]));
        return u0;
    }

    static double power_eig(const Mat& Hm, int it = 60) {
        int n = Hm.r;
        std::vector<double> v(n, 1.0 / std::sqrt((double)n));
        double lam = 0;
        for (int k = 0; k < it; ++k) {
            std::vector<double> w(n, 0.0);
            for (int i = 0; i < n; ++i) {
                double s = 0;
                for (int j = 0; j < n; ++j) s += Hm(i, j) * v[j];
                w[i] = s;
            }
            double nrm = 0; for (double x : w) nrm += x * x; nrm = std::sqrt(nrm);
            if (nrm < 1e-12) break;
            for (int i = 0; i < n; ++i) v[i] = w[i] / nrm;
            lam = nrm;
        }
        return lam;
    }
};

}  // namespace ctrl
