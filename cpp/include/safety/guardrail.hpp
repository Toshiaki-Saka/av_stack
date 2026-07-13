#pragma once
#include "world/scenario.hpp"   // LANE constant

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ad::safety {

// ---------------------------------------------------------------------------
// Minimal track view consumed by the guardrail.
// id, x, y, vx, vy — matches the tuple produced by world_cpp.MultiObjectTracker.
// ---------------------------------------------------------------------------
struct TrackEntry {
    int    id;
    double x, y, vx, vy;
    bool   confirmed = true;
};

struct GuardrailResult {
    double      a;       // longitudinal acceleration override [m/s²]
    double      delta;   // steering override [rad]
    std::string status;  // "OK" | "OVERRIDE"
    std::string reason;
};

class Guardrail {
public:
    double rho;          // reaction time [s]
    double a_accel_max;  // comfortable accel [m/s²]
    double b_min;        // min braking capability [m/s²]
    double b_max_lead;   // assumed lead braking [m/s²]
    double b_emergency;  // ego emergency brake [m/s²]
    double ttc_min;      // minimum TTC threshold [s]
    int    hold;         // override hold duration [steps]
    bool   use_lateral;  // enable lateral (cut-in) RSS check

    // Lateral RSS parameters (Shalev-Shwartz lateral safe distance). These are
    // distinct from the longitudinal ones: a lane change is a far gentler
    // manoeuvre than braking, so reusing a_accel_max / b_min here would model
    // the wrong worst case.
    double mu;           // lateral clearance buffer [m]
    double a_lat_max;    // worst-case lateral acceleration [m/s²]
    double b_lat_min;    // min lateral braking capability [m/s²]
    double veh_len;      // vehicle length, for the longitudinal RSS band [m]

    static constexpr double _MAX_VY = 3.0;  // ghost-track lateral speed filter

    explicit Guardrail(double rho_         = 0.4,
                       double a_accel_max_ = 1.0,
                       double b_min_       = 4.0,
                       double b_max_lead_  = 8.0,
                       double b_emergency_ = 6.0,
                       double ttc_min_     = 2.5,
                       int    hold_        = 15,
                       bool   use_lateral_ = true,
                       double mu_          = 0.5,
                       double a_lat_max_   = 0.5,
                       double b_lat_min_   = 1.0,
                       double veh_len_     = 4.5)
        : rho(rho_), a_accel_max(a_accel_max_),
          b_min(b_min_), b_max_lead(b_max_lead_),
          b_emergency(b_emergency_), ttc_min(ttc_min_),
          hold(hold_), use_lateral(use_lateral_),
          mu(mu_), a_lat_max(a_lat_max_), b_lat_min(b_lat_min_),
          veh_len(veh_len_), _hold_counter(0) {}

    // RSS longitudinal safe distance [m]
    double rss_min_distance(double v_ego, double v_lead) const {
        double s_star = v_ego * rho
            + 0.5 * a_accel_max * rho * rho
            + (v_ego + rho * a_accel_max) * (v_ego + rho * a_accel_max)
              / (2.0 * b_min)
            - v_lead * v_lead / (2.0 * b_max_lead);
        return std::max(0.0, s_star);
    }

    // Lateral distance a vehicle covers if it accelerates laterally at a_lat_max for
    // rho, then brakes at b_lat_min to a lateral stop (the RSS worst case).
    double lat_travel(double v_toward) const {
        double v = std::max(0.0, v_toward);
        return v * rho
            + 0.5 * a_lat_max * rho * rho
            + (v + rho * a_lat_max) * (v + rho * a_lat_max) / (2.0 * b_lat_min);
    }

    // RSS lateral safe distance [m] given the other vehicle's lateral speed.
    // Ego is treated as laterally stationary; both parties react for rho and then
    // brake laterally. Below this distance (plus buffer mu) the lateral situation
    // is unsafe. See docs/TECHNICAL.md 9.3.
    double rss_lateral_min_distance(double v_other_lat) const {
        return mu + lat_travel(0.0) + lat_travel(std::abs(v_other_lat));
    }

    // Evaluate one guardrail check step.
    // ego  = [x, y, vx, vy]
    // cmd  = [a, delta] — proposed control
    // tracks = list of tracked objects
    GuardrailResult check(const double ego[4],
                          const double cmd[2],
                          const std::vector<TrackEntry>& tracks)
    {
        double v_ego = std::hypot(ego[2], ego[3]);
        const std::string* reason_ptr = nullptr;
        std::string reason;
        bool trigger = false;

        // --- Confirmed lead in ego lane ---
        const TrackEntry* lead = _lead_in_lane(ego, tracks);
        if (lead) {
            double gap = lead->x - ego[0];
            double d_rss = rss_min_distance(v_ego, std::max(0.0, lead->vx));
            if (gap < d_rss) { trigger = true; reason = "RSS_LONGITUDINAL"; }

            double rel_v = v_ego - std::max(0.0, lead->vx);
            double ttc   = (rel_v > 0.01) ? gap / rel_v : 1e9;
            if (ttc < ttc_min) { trigger = true; reason = "TTC"; }
        }

        // --- Cut-in threat (optional lateral RSS) ---
        if (use_lateral && _cut_in_threat(ego, tracks)) {
            trigger = true;
            reason  = "CUT_IN";
        }

        // --- Latch ---
        if (trigger) _hold_counter = hold;
        bool active = (_hold_counter > 0);
        if (_hold_counter > 0) --_hold_counter;

        if (!active) {
            return { cmd[0], cmd[1], "OK", "" };
        }

        double a_safe  = -b_emergency;
        double d_safe  = 0.0;
        return { a_safe, d_safe, "OVERRIDE", reason };
    }

private:
    int _hold_counter;

    // Closest confirmed object ahead in ego lane (|vy| filter to reject ghosts)
    const TrackEntry* _lead_in_lane(const double ego[4],
                                    const std::vector<TrackEntry>& tracks) const
    {
        const TrackEntry* best = nullptr;
        double best_gap = 1e9;
        for (const auto& t : tracks) {
            if (!t.confirmed) continue;
            if (std::abs(t.vy) > _MAX_VY) continue;
            if (std::abs(t.y - ego[1]) > ad::world::LANE * 0.5) continue;
            double gap = t.x - ego[0];
            if (gap > 0 && gap < best_gap) { best_gap = gap; best = &t; }
        }
        return best;
    }

    // Returns true if any object poses a cut-in RSS violation
    bool _cut_in_threat(const double ego[4],
                        const std::vector<TrackEntry>& tracks) const
    {
        double v_ego = std::hypot(ego[2], ego[3]);
        for (const auto& t : tracks) {
            if (!t.confirmed) continue;
            // Ghost tracks (sensor artifacts) often carry a large vy estimate, which
            // inflates the lateral RSS distance until the gap test passes trivially.
            if (std::abs(t.vy) > _MAX_VY) continue;

            // Lateral: moving into ego lane
            double dy = t.y - ego[1];
            if (std::abs(dy) > ad::world::LANE) continue;
            // A neighbour drifting away from the ego is not a cut-in; requiring vy to
            // point toward the ego keeps a departing vehicle from latching the brake.
            if (dy * t.vy >= -0.05) continue;
            double d_lat_min = rss_lateral_min_distance(t.vy);
            if (std::abs(dy) >= d_lat_min) continue;

            // Longitudinal: inside the RSS band. The band extends one vehicle length
            // behind the ego, because a neighbour merging alongside is a hazard even
            // when its nose has not yet passed the ego's.
            double gap   = t.x - ego[0];
            double d_rss = rss_min_distance(v_ego, std::max(0.0, t.vx));
            if (-veh_len <= gap && gap <= d_rss + veh_len) return true;
        }
        return false;
    }
};

}  // namespace ad::safety
