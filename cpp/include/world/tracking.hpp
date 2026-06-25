#pragma once
// World model: multi-object tracking with an Interacting Multiple Model (IMM)
// Kalman filter and short-horizon prediction.
//
// Each track runs an IMM over two constant-velocity models that differ only in
// process noise: a low-noise "cruise" model and a high-noise "manoeuvre" model.
// The IMM blends them by likelihood, so a track follows a smoothly cruising agent
// tightly yet reacts quickly when it brakes or swerves; the manoeuvre-mode
// probability is itself a useful signal downstream. State = [x, y, vx, vy].
#include <vector>
#include <cmath>
#include <map>
#ifndef M_PI
static constexpr double M_PI = 3.14159265358979323846;
#endif
#include "control/linalg.hpp"

namespace world {
using ctrl::Mat;
using ctrl::T;
using ctrl::inv;

struct Gaussian { Mat x; Mat P; };

// --- constant-velocity Kalman filter with tunable process-noise density ---
struct KFCV {
    Mat x{4, 1}, P;
    double sigma_a;  // process-noise (acceleration) std

    KFCV(double sa = 1.0) : sigma_a(sa) { P = ctrl::Mat::eye(4); for (int i = 2; i < 4; ++i) P(i, i) = 100; }

    static Mat F(double dt) {
        Mat f = Mat::eye(4); f(0, 2) = dt; f(1, 3) = dt; return f;
    }
    Mat Q(double dt) const {
        double s = sigma_a * sigma_a;
        double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
        Mat q(4, 4);
        q(0, 0) = q(1, 1) = dt4 / 4 * s;
        q(0, 2) = q(2, 0) = q(1, 3) = q(3, 1) = dt3 / 2 * s;
        q(2, 2) = q(3, 3) = dt2 * s;
        return q;
    }
    static Mat H() { Mat h(2, 4); h(0, 0) = 1; h(1, 1) = 1; return h; }

    void predict(double dt) {
        Mat f = F(dt);
        x = f * x;
        P = f * P * T(f) + Q(dt);
    }
    // returns Gaussian innovation likelihood for IMM weighting
    double update(const Mat& z, const Mat& R) {
        Mat h = H();
        Mat y = z - h * x;
        Mat S = h * P * T(h) + R;
        Mat Si = inv(S);
        Mat K = P * T(h) * Si;
        x = x + K * y;
        P = (Mat::eye(4) - K * h) * P;
        double d2 = (T(y) * Si * y)(0, 0);
        double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
        if (det < 1e-12) det = 1e-12;
        return std::exp(-0.5 * d2) / (2 * M_PI * std::sqrt(det));
    }
};

// --- IMM over two CV models (cruise / manoeuvre) ---
struct IMM {
    KFCV cruise{0.5}, maneuver{6.0};
    double mu0 = 0.5, mu1 = 0.5;        // mode probabilities
    double p_stay = 0.95;               // mode-transition self-probability

    void init(double x, double y, double vx, double vy) {
        for (KFCV* f : {&cruise, &maneuver}) {
            f->x = Mat(4, 1); f->x(0, 0) = x; f->x(1, 0) = y; f->x(2, 0) = vx; f->x(3, 0) = vy;
            f->P = Mat::eye(4); f->P(2, 2) = f->P(3, 3) = 25;
        }
    }
    // mixing + per-model prediction
    void predict(double dt) {
        double Pi[2][2] = {{p_stay, 1 - p_stay}, {1 - p_stay, p_stay}};
        double mu[2] = {mu0, mu1};
        double cbar[2] = {0, 0};
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) cbar[j] += Pi[i][j] * mu[i];
        KFCV* f[2] = {&cruise, &maneuver};
        Mat x0[2], P0[2];
        for (int j = 0; j < 2; ++j) {
            Mat xm(4, 1), Pm(4, 4);
            double w[2];
            for (int i = 0; i < 2; ++i) w[i] = Pi[i][j] * mu[i] / (cbar[j] + 1e-12);
            for (int i = 0; i < 2; ++i) xm = xm + ctrl::scale(f[i]->x, w[i]);
            for (int i = 0; i < 2; ++i) {
                Mat d = f[i]->x - xm;
                Pm = Pm + ctrl::scale(f[i]->P + d * T(d), w[i]);
            }
            x0[j] = xm; P0[j] = Pm;
        }
        for (int j = 0; j < 2; ++j) { f[j]->x = x0[j]; f[j]->P = P0[j]; f[j]->predict(dt); }
    }
    void update(const Mat& z, const Mat& R) {
        double L0 = cruise.update(z, R), L1 = maneuver.update(z, R);
        double Pi[2][2] = {{p_stay, 1 - p_stay}, {1 - p_stay, p_stay}};
        double mu[2] = {mu0, mu1}, cbar[2] = {0, 0};
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) cbar[j] += Pi[i][j] * mu[i];
        double a0 = L0 * cbar[0], a1 = L1 * cbar[1], s = a0 + a1 + 1e-300;
        mu0 = a0 / s; mu1 = a1 / s;
    }
    // predict without measurement (for missed tracks)
    void predict_only(double dt) { predict(dt); }

    Gaussian estimate() const {
        Mat x = ctrl::scale(cruise.x, mu0) + ctrl::scale(maneuver.x, mu1);
        Mat P = ctrl::scale(cruise.P, mu0) + ctrl::scale(maneuver.P, mu1);
        Mat d0 = cruise.x - x, d1 = maneuver.x - x;
        P = P + ctrl::scale(d0 * T(d0), mu0) + ctrl::scale(d1 * T(d1), mu1);
        return {x, P};
    }
    double p_maneuver() const { return mu1; }
};

struct Track {
    int id;
    IMM imm;
    int hits = 1, misses = 0;
    bool confirmed = false;
};

struct MultiObjectTracker {
    double dt = 0.1;
    double gate = 9.21;       // chi-square 2-DOF 99%
    int min_hits = 3, max_miss = 5;
    std::vector<Track> tracks;
    int next_id = 1;

    struct Det { double zx, zy; Mat R; bool has_v; double vx, vy; };

    // one cycle: predict, associate (greedy Mahalanobis), update, manage lifecycle
    void step(std::vector<Det>& dets) {
        for (auto& t : tracks) t.imm.predict(dt);

        int nt = (int)tracks.size(), nd = (int)dets.size();
        std::vector<int> tmatch(nt, -1), dmatch(nd, -1);
        std::vector<std::tuple<double, int, int>> pairs;
        Mat H = KFCV::H();
        for (int i = 0; i < nt; ++i) {
            Gaussian g = tracks[i].imm.estimate();
            Mat zp = H * g.x;
            for (int j = 0; j < nd; ++j) {
                Mat S = H * g.P * T(H) + dets[j].R;
                Mat y(2, 1); y(0, 0) = dets[j].zx - zp(0, 0); y(1, 0) = dets[j].zy - zp(1, 0);
                double d2 = (T(y) * inv(S) * y)(0, 0);
                if (d2 < gate) pairs.emplace_back(d2, i, j);
            }
        }
        std::sort(pairs.begin(), pairs.end());
        for (auto& [d2, i, j] : pairs) {
            if (tmatch[i] >= 0 || dmatch[j] >= 0) continue;
            tmatch[i] = j; dmatch[j] = i;
        }
        for (int i = 0; i < nt; ++i) {
            if (tmatch[i] >= 0) {
                Det& d = dets[tmatch[i]];
                Mat z(2, 1); z(0, 0) = d.zx; z(1, 0) = d.zy;
                tracks[i].imm.update(z, d.R);
                tracks[i].hits++; tracks[i].misses = 0;
                if (tracks[i].hits >= min_hits) tracks[i].confirmed = true;
            } else {
                tracks[i].misses++;
            }
        }
        // spawn new tracks for unmatched detections
        for (int j = 0; j < nd; ++j) {
            if (dmatch[j] >= 0) continue;
            Track t; t.id = next_id++;
            double vx = dets[j].has_v ? dets[j].vx : 0.0;
            double vy = dets[j].has_v ? dets[j].vy : 0.0;
            t.imm.init(dets[j].zx, dets[j].zy, vx, vy);
            tracks.push_back(t);
        }
        // delete stale
        std::vector<Track> keep;
        for (auto& t : tracks) if (t.misses <= max_miss) keep.push_back(t);
        tracks.swap(keep);
    }
};

}  // namespace world
