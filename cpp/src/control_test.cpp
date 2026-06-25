#include <cstdio>
#include <cmath>
#include <vector>
#include "control/bicycle.hpp"
#include "control/lqr.hpp"
#include "control/mpc.hpp"
using namespace ctrl;

static int fails = 0;
static void check(bool ok, const char* msg) {
    std::printf("  [%s] %s\n", ok ? "OK" : "FAIL", msg);
    if (!ok) ++fails;
}

int main() {
    Bicycle car; car.L = 2.7;
    double dt = 0.1;

    // 1. RK4 sanity: straight line and constant-steer circle
    {
        Bicycle::State s = {0, 0, 0, 10};
        for (int k = 0; k < 50; ++k) s = car.step(s, {0, 0}, dt);
        check(std::fabs(s[0] - 50.0) < 1e-3 && std::fabs(s[1]) < 1e-9, "straight: x=50, y=0");

        s = {0, 0, 0, 10};
        double delta = 0.1;                      // constant steer -> circle, R = L/tan(delta)
        double R = car.L / std::tan(delta);
        for (int k = 0; k < 30; ++k) s = car.step(s, {0, delta}, dt);
        // turn rate omega = v/R; after time T heading = omega*T
        double T = 30 * dt, omega = 10.0 / R;
        check(std::fabs(s[2] - omega * T) < 1e-2, "circle: heading matches v/R*T");
    }

    // 2. Linearisation is a consistent 2nd-order model of the nonlinear step
    {
        Bicycle::State s0 = {3, -1, 0.2, 9};
        Bicycle::Input u0 = {0.5, 0.05};
        Mat A, B; car.linearize(s0, u0, dt, A, B);
        double worst = 0;
        for (int trial = 0; trial < 20; ++trial) {
            Bicycle::State ds; Bicycle::Input du;
            double e = 1e-3;
            for (int i = 0; i < 4; ++i) ds[i] = ((trial * 7 + i) % 5 - 2) * e;
            for (int i = 0; i < 2; ++i) du[i] = ((trial * 3 + i) % 5 - 2) * e;
            Bicycle::State sp = {s0[0]+ds[0], s0[1]+ds[1], s0[2]+ds[2], s0[3]+ds[3]};
            Bicycle::Input up = {u0[0]+du[0], u0[1]+du[1]};
            Bicycle::State f_nl = car.step(sp, up, dt);
            Bicycle::State f0 = car.step(s0, u0, dt);
            for (int i = 0; i < 4; ++i) {
                double lin = f0[i];
                for (int j = 0; j < 4; ++j) lin += A(i, j) * ds[j];
                for (int j = 0; j < 2; ++j) lin += B(i, j) * du[j];
                worst = std::max(worst, std::fabs(f_nl[i] - lin));
            }
        }
        std::printf("  linearisation worst residual (O(eps^2)) = %.2e\n", worst);
        check(worst < 1e-5, "linearisation matches nonlinear step to 2nd order");
    }

    // 3. LQR makes the lateral error closed loop stable and decaying
    {
        LateralLQR lat; lat.L = car.L; lat.dt = dt;
        double v = 10;
        // build the same discrete error model and check spectral radius of A-BK
        Mat A(2, 2), B(2, 1), Q(2, 2), R(1, 1);
        A(0,0)=1; A(0,1)=v*dt; A(1,0)=0; A(1,1)=1; B(1,0)=(v/car.L)*dt;
        Q(0,0)=lat.q_ey; Q(1,1)=lat.q_epsi; R(0,0)=lat.r_delta;
        Mat K = dlqr(A, B, Q, R);
        Mat Acl = A - B * K;
        // simulate error decay from e=(1.5, 0)
        double ey = 1.5, epsi = 0.0;
        for (int k = 0; k < 80; ++k) {
            double delta = -(K(0,0)*ey + K(0,1)*epsi);
            double ey_n = Acl(0,0)*ey + Acl(0,1)*epsi;
            double epsi_n = Acl(1,0)*ey + Acl(1,1)*epsi;
            ey = ey_n; epsi = epsi_n; (void)delta;
        }
        std::printf("  LQR residual cross-track after 8s = %.3e m\n", std::fabs(ey));
        check(std::fabs(ey) < 1e-2, "LQR drives cross-track error to ~0");
    }

    // 4. MPC: speed tracking with acceleration limit, and lateral recovery
    {
        MPC mpc; mpc.model = car; mpc.dt = dt; mpc.N = 15;
        int n = 4, m = 2;
        auto make_ref = [&](double y_ref, double v_ref) {
            std::vector<double> sref((mpc.N + 1) * n), uref(mpc.N * m, 0.0);
            for (int k = 0; k <= mpc.N; ++k) {
                sref[k*n+0] = v_ref * dt * k; sref[k*n+1] = y_ref;
                sref[k*n+2] = 0; sref[k*n+3] = v_ref;
            }
            return std::make_pair(sref, uref);
        };
        // speed tracking 8 -> 12
        Bicycle::State s = {0, 0, 0, 8};
        double max_a = 0;
        for (int step = 0; step < 60; ++step) {
            auto pr = make_ref(0.0, 12.0);
            // shift reference x to current
            for (int k = 0; k <= mpc.N; ++k) pr.first[k*n+0] += s[0];
            auto u = mpc.solve(s, pr.first, pr.second);
            max_a = std::max(max_a, std::fabs(u[0]));
            s = car.step(s, u, dt);
        }
        std::printf("  MPC speed: v=%.2f (target 12), max|a|=%.2f (limit 3)\n", s[3], max_a);
        check(std::fabs(s[3] - 12.0) < 0.3, "MPC reaches target speed");
        check(max_a <= mpc.a_max + 1e-6, "MPC respects acceleration limit");

        // lateral recovery from 1.5 m offset
        s = {0, 1.5, 0, 10};
        double max_d = 0;
        for (int step = 0; step < 80; ++step) {
            auto pr = make_ref(0.0, 10.0);
            for (int k = 0; k <= mpc.N; ++k) pr.first[k*n+0] += s[0];
            auto u = mpc.solve(s, pr.first, pr.second);
            max_d = std::max(max_d, std::fabs(u[1]));
            s = car.step(s, u, dt);
        }
        std::printf("  MPC lateral: y=%.3f (target 0), max|delta|=%.3f (limit 0.6)\n", s[1], max_d);
        check(std::fabs(s[1]) < 0.15, "MPC recovers to lane center");
        check(max_d <= mpc.delta_max + 1e-6, "MPC respects steering limit");
    }

    std::printf(fails ? "CONTROL TEST: %d FAIL\n" : "CONTROL TEST PASS\n", fails);
    return fails ? 1 : 0;
}
