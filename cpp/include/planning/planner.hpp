#pragma once
#include "planning/trajectory.hpp"
#include "world/scenario.hpp"    // LANE

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ad::planning {

// Predicted position sequence for one tracked object — plain (x,y) at each step.
// Matches mot.predict() output from world_cpp.MultiObjectTracker.
struct PredictedTrack {
    std::vector<std::pair<double,double>> positions;  // (x,y) at horizon steps 0,1,...
};

struct PlannerResult {
    Trajectory  traj;
    std::string behavior;  // "CRUISE" | "FOLLOW/SLOW" | "CHANGE_LANE" | "EMERGENCY_SLOW"
};

class Planner {
public:
    double dt, v_des, L, safe_radius, t_change;
    int    M;
    double veh_len;
    std::vector<double> lanes;
    bool   fault;  // simulates a degraded channel (ignores traffic)

    // IDM parameters (match Python defaults)
    double idm_a, idm_b, idm_T, idm_s0;

    explicit Planner(double dt_          = 0.1,
                     double horizon      = 4.0,
                     double v_des_       = 13.0,
                     double L_           = 2.7,
                     double safe_radius_ = 4.5,
                     double t_change_    = 3.0,
                     std::vector<double> lanes_ = {0.0, ad::world::LANE},
                     bool   fault_       = false)
        : dt(dt_), v_des(v_des_), L(L_),
          safe_radius(safe_radius_), t_change(t_change_),
          M(static_cast<int>(horizon / dt_)),
          veh_len(4.5),
          lanes(std::move(lanes_)),
          fault(fault_),
          idm_a(1.5), idm_b(2.0), idm_T(1.5), idm_s0(5.0),
          _committed_lane(-1e9), _has_commit(false),
          _committed_traj{}
    {
        if (fault_) {
            idm_T = 0.2; idm_b = 1.0; idm_s0 = 2.0;
            safe_radius = 0.0;
        }
    }

    // ego = [x, y, psi, v].  preds = constant-velocity predicted tracks from tracker.
    PlannerResult plan(const double ego[4],
                       const std::vector<PredictedTrack>& preds)
    {
        if (fault) {
            double v0 = std::max(ego[3], v_des);
            std::vector<double> xs(M+1), ys(M+1, ego[1]), vs(M+1, v0);
            for (int k = 0; k <= M; ++k) xs[k] = ego[0] + v0 * k * dt;
            return { Trajectory(xs, ys, vs, dt, L), "CRUISE" };
        }

        double ego_lane = lanes[0];
        for (double l : lanes)
            if (std::abs(l - ego[1]) < std::abs(ego_lane - ego[1])) ego_lane = l;

        // Mid-maneuver: hold committed trajectory until arrival
        if (_has_commit && std::abs(ego[1] - _committed_lane) >= 0.25) {
            auto beh = _behaviour(ego, _committed_lane, ego_lane, _committed_traj);
            return { _committed_traj, beh };
        }
        if (_has_commit && std::abs(ego[1] - _committed_lane) < 0.25) {
            _has_commit = false;
            _committed_traj = {};
        }

        Trajectory best_traj;
        std::string best_beh = "EMERGENCY_SLOW";
        double best_cost = std::numeric_limits<double>::max();
        double best_target = ego_lane;

        for (double target_lane : lanes) {
            Trajectory traj = _gen(ego, target_lane, preds);
            auto cost_opt   = _cost(traj, preds, target_lane, ego_lane);
            if (!cost_opt.has_value()) continue;
            std::string beh = _behaviour(ego, target_lane, ego_lane, traj);
            if (*cost_opt < best_cost) {
                best_cost   = *cost_opt;
                best_traj   = std::move(traj);
                best_beh    = beh;
                best_target = target_lane;
            }
        }

        if (best_cost == std::numeric_limits<double>::max()) {
            // All lanes unsafe — decelerate in current lane
            std::vector<double> xs(M+1), ys(M+1, ego[1]), vs(M+1);
            xs[0] = ego[0]; vs[0] = ego[3];
            for (int k = 0; k < M; ++k) {
                vs[k+1] = std::max(0.0, vs[k] - idm_b * 1.5 * dt);
                xs[k+1] = xs[k] + vs[k] * dt;
            }
            return { Trajectory(xs, ys, vs, dt, L), "EMERGENCY_SLOW" };
        }

        // Commit if changing lane
        if (!_has_commit && std::abs(best_target - ego_lane) > 0.1) {
            _committed_lane = best_target;
            _committed_traj = best_traj;
            _has_commit     = true;
        }
        return { best_traj, best_beh };
    }

    double rss_min_distance(double v_ego, double v_lead) const {
        double rho = 0.4, a_max = 1.0, b = 4.0, b_lead = 8.0;
        return std::max(0.0,
            v_ego * rho + 0.5 * a_max * rho * rho
            + (v_ego + rho * a_max) * (v_ego + rho * a_max) / (2.0 * b)
            - v_lead * v_lead / (2.0 * b_lead));
    }

private:
    double     _committed_lane;
    bool       _has_commit;
    Trajectory _committed_traj;

    double _idm_accel(double v, double gap, double v_lead) const {
        double dv     = v - v_lead;
        double s_star = idm_s0 + std::max(0.0,
            v * idm_T + v * dv / (2.0 * std::sqrt(idm_a * idm_b)));
        gap = std::max(gap, 0.1);
        double a = idm_a * (1.0 - std::pow(v / std::max(v_des, 0.1), 4.0)
                                - (s_star / gap) * (s_star / gap));
        return std::max(-idm_b * 1.5, std::min(idm_a, a));
    }

    // Nearest predicted lead in target_lane at horizon step k.
    // Returns {gap, v_lead} or nullopt.  Velocity estimated from consecutive positions.
    std::optional<std::array<double,2>>
    _lead_in_lane(const std::vector<PredictedTrack>& preds,
                  double x_ego, double target_lane, int k) const
    {
        double best_gap = 1e9, best_v = 0.0;
        bool   found    = false;
        for (const auto& p : preds) {
            int j  = std::min(k,   static_cast<int>(p.positions.size()) - 1);
            if (j < 0) continue;
            double px = p.positions[j].first, py = p.positions[j].second;
            if (std::abs(py - target_lane) > ad::world::LANE * 0.5) continue;
            if (px <= x_ego) continue;
            double gap = px - x_ego - veh_len;
            int    jp  = std::max(0, j - 1);
            double v_lead = (j > 0)
                ? (p.positions[j].first - p.positions[jp].first) / std::max(dt, 1e-6)
                : 0.0;
            if (gap < best_gap) { best_gap = gap; best_v = v_lead; found = true; }
        }
        if (!found) return std::nullopt;
        return std::array<double,2>{ best_gap, best_v };
    }

    Trajectory _gen(const double ego[4], double target_lane,
                    const std::vector<PredictedTrack>& preds) const
    {
        std::vector<double> xs(M+1), ys(M+1), vs(M+1);
        xs[0] = ego[0];
        double y0 = ego[1];
        vs[0]     = ego[3];

        for (int k = 0; k < M; ++k) {
            auto lead = _lead_in_lane(preds, xs[k], target_lane, k);
            double a;
            if (lead.has_value())
                a = _idm_accel(vs[k], (*lead)[0], std::max(0.0, (*lead)[1]));
            else
                a = _idm_accel(vs[k], 1e6, vs[k]);

            vs[k+1] = std::max(0.0, vs[k] + a * dt);
            xs[k+1] = xs[k] + vs[k] * dt;
        }

        for (int k = 0; k <= M; ++k) {
            double t = k * dt;
            ys[k] = y0 + (target_lane - y0) * smoothstep(t, 0.0, t_change);
        }
        return Trajectory(xs, ys, vs, dt, L);
    }

    std::optional<double>
    _cost(const Trajectory& traj,
          const std::vector<PredictedTrack>& preds,
          double target_lane, double ego_lane) const
    {
        double min_clear = 1e9;
        bool   unsafe    = false;
        for (const auto& p : preds) {
            int steps = std::min(traj.n, static_cast<int>(p.positions.size()));
            for (int k = 0; k < steps; ++k) {
                double dx = std::abs(traj.x[k] - p.positions[k].first);
                double dy = std::abs(traj.y[k] - p.positions[k].second);
                double d  = std::hypot(dx, dy);
                min_clear = std::min(min_clear, d);
                if (dx < safe_radius && dy < ad::world::LANE * 0.5)
                    unsafe = true;
            }
        }
        if (unsafe) return std::nullopt;

        double v_mean = 0.0, lat_acc_max = 0.0;
        for (int k = 0; k < traj.n; ++k) {
            double dv = traj.v[k] - v_des;
            v_mean += dv * dv;
            lat_acc_max = std::max(lat_acc_max,
                traj.v[k] * traj.v[k] * std::abs(traj.kappa[k]));
        }
        v_mean /= traj.n;
        double progress = traj.x.empty() ? 0.0 : -(traj.x.back() - traj.x.front()) * 0.4;
        double lane_change_cost = (std::abs(target_lane - ego_lane) > 0.1) ? 6.0 : 0.0;
        double right_pref = std::abs(target_lane) * 1.0;
        double margin_cost = 30.0 / std::max(min_clear, 1e-3);

        return v_mean + progress + lat_acc_max * 4.0
             + lane_change_cost + right_pref + margin_cost;
    }

    std::string _behaviour(const double ego[4], double target_lane,
                           double ego_lane, const Trajectory& traj) const
    {
        if (std::abs(target_lane - ego_lane) > 0.1) return "CHANGE_LANE";
        if (!traj.v.empty() && traj.v.back() < v_des - 0.5) return "FOLLOW/SLOW";
        return "CRUISE";
    }
};

}  // namespace ad::planning
