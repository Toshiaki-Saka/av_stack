#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace ad::world {

static constexpr double LANE = 3.5;  // lane width [m]

struct BrakeSchedule { double t_start, decel; };
struct CutIn        { double t_start, t_end, y_target; };

struct Agent {
    int    id;
    double x, y, vx, vy;
    std::string kind;

    bool         has_brake  = false;
    BrakeSchedule brake{};

    bool  has_cut_in = false;
    CutIn cut_in{};

    double t = 0.0;

    Agent(int id_, double x_, double y_, double vx_, double vy_ = 0.0,
          std::string kind_ = "car")
        : id(id_), x(x_), y(y_), vx(vx_), vy(vy_), kind(std::move(kind_)) {}

    void set_brake(double t_start, double decel) {
        has_brake = true;
        brake     = {t_start, decel};
    }
    void set_cut_in(double t_start, double t_end, double y_target) {
        has_cut_in = true;
        cut_in     = {t_start, t_end, y_target};
    }

    void step(double dt) {
        double ax = 0.0;
        if (has_brake && t >= brake.t_start)
            ax = -brake.decel;

        double vy_des = 0.0;
        if (has_cut_in && cut_in.t_start <= t && t <= cut_in.t_end)
            vy_des = 2.0 * std::tanh((cut_in.y_target - y) * 0.8);

        x  += vx * dt;
        y  += vy * dt;
        vx  = std::max(0.0, vx + ax * dt);
        vy += (vy_des - vy) * std::min(1.0, 4.0 * dt);
        t  += dt;
    }
};

struct ObjectEntry {
    int id;
    double x, y, vx, vy;
    std::string kind;
};

struct World {
    std::vector<Agent> agents;

    explicit World(std::vector<Agent> agents_) : agents(std::move(agents_)) {}

    void step(double dt) {
        for (auto& a : agents) a.step(dt);
    }

    std::vector<ObjectEntry> true_objects() const {
        std::vector<ObjectEntry> out;
        out.reserve(agents.size());
        for (const auto& a : agents)
            out.push_back({a.id, a.x, a.y, a.vx, a.vy, a.kind});
        return out;
    }
};

// ---------------------------------------------------------------------------
// Scenario factories
// ---------------------------------------------------------------------------

inline World scenario_lead_brake() {
    Agent a(1, 45.0, 0.0, 8.0, 0.0, "car");
    a.set_brake(3.0, 4.0);
    return World({a});
}

inline World scenario_hard_brake() {
    Agent a(1, 26.0, 0.0, 12.0, 0.0, "car");
    a.set_brake(2.0, 8.0);
    return World({a});
}

inline World scenario_cut_in() {
    Agent a(2, 18.0, LANE, 9.0, 0.0, "car");
    a.set_cut_in(0.5, 3.5, 0.0);
    return World({a});
}

inline World scenario_mixed() {
    Agent a1(1, 40.0, 0.0,    7.0, 0.0, "car");
    Agent a2(3, 70.0, LANE,  13.0, 0.0, "car");
    return World({a1, a2});
}

inline World scenario_acc_follow() {
    Agent a(1, 35.0, 0.0, 8.0, 0.0, "car");
    return World({a});
}

inline World scenario_lane_avoidance() {
    Agent a(1, 45.0, 0.0, 3.0, 0.0, "car");
    return World({a});
}

inline World scenario_safety_stop() {
    Agent a1(1, 25.0, 0.0,   12.0, 0.0, "car");
    a1.set_brake(1.5, 8.0);
    Agent a2(2,  5.0, LANE,  10.0, 0.0, "car");
    return World({a1, a2});
}

}  // namespace ad::world
