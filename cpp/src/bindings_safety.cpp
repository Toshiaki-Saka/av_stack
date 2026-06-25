// pybind11 adapter for ad::safety::SpeedGovernor (ad-pipeline-demo).
//
// Python interface (unchanged from previous standalone impl):
//   gov = SpeedGovernor()
//   capped = gov.cap_speed(target_v, obstacles, ego_x, ego_y, ego_yaw)
//     obstacles: list of [x, y, vx, vy]
//   gov.set_params(T_horizon, dt, decel_max, margin)
//
// The adapter synthesises an ad::common::Plan (straight-ahead path) from the
// ego pose, then delegates to ad::safety::SpeedGovernor::cap_speed().
#include <array>
#include <cmath>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ad/safety/speed_governor.hpp"

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Adapter — maps the av_stack Python interface to ad::safety::SpeedGovernor
// ---------------------------------------------------------------------------
class SpeedGovernorAdapter {
 public:
  using AdParams = ad::safety::SpeedGovernor::Params;

  SpeedGovernorAdapter() : gov_{default_params()} {}

  // obstacles: each element is [x, y, vx, vy]
  double cap_speed(double target_v,
                   const std::vector<std::array<double, 4>>& obstacles,
                   double ego_x, double ego_y, double ego_yaw) const {
    // Build ego VehicleState
    ad::common::VehicleState ego;
    ego.pose.x = ego_x; ego.pose.y = ego_y; ego.pose.yaw = ego_yaw;
    ego.velocity = target_v;

    // Synthesise a straight-ahead plan from ego heading
    ad::common::Plan plan;
    plan.target_speed = target_v;
    const double react = gov_params_.react_distance;
    const int n_pts = 20;
    for (int i = 0; i <= n_pts; ++i) {
      const double s = react * i / n_pts;
      ad::common::Pose2D pt;
      pt.x   = ego_x + s * std::cos(ego_yaw);
      pt.y   = ego_y + s * std::sin(ego_yaw);
      pt.yaw = ego_yaw;
      plan.path.push_back(pt);
    }

    // Convert Python obstacles → ad::common::Obstacle
    std::vector<ad::common::Obstacle> obs_vec;
    obs_vec.reserve(obstacles.size());
    for (const auto& o : obstacles) {
      ad::common::Obstacle ob;
      ob.position.x = o[0]; ob.position.y = o[1];
      ob.velocity.x = o[2]; ob.velocity.y = o[3];
      ob.radius = 1.0;   // default vehicle radius proxy
      obs_vec.push_back(ob);
    }

    return gov_.cap_speed(ego, plan, obs_vec, target_v);
  }

  void set_params(double T_horizon, double /*dt*/, double decel_max, double margin) {
    gov_params_.horizon_time  = T_horizon;
    gov_params_.max_decel     = decel_max;
    gov_params_.safety_margin = margin;
    gov_ = ad::safety::SpeedGovernor{gov_params_};
  }

 private:
  static AdParams default_params() {
    AdParams p;
    p.horizon_time   = 3.0;
    p.react_distance = 18.0;
    p.vehicle_radius = 1.5;
    p.safety_margin  = 1.0;
    p.brake_margin   = 2.0;
    p.max_decel      = 4.0;
    p.min_speed_eps  = 1.0;
    return p;
  }

  AdParams gov_params_{default_params()};
  ad::safety::SpeedGovernor gov_{default_params()};
};

// ---------------------------------------------------------------------------
// pybind11 module
// ---------------------------------------------------------------------------
PYBIND11_MODULE(safety_cpp, mod) {
  mod.doc() = "Predictive speed governor — wraps ad::safety::SpeedGovernor "
              "(ad-pipeline-demo shared implementation).";

  py::class_<SpeedGovernorAdapter>(mod, "SpeedGovernor")
      .def(py::init<>())
      .def("cap_speed",
           [](const SpeedGovernorAdapter& g, double tv,
              const std::vector<std::array<double, 4>>& obs,
              double ex, double ey, double eyaw) {
             return g.cap_speed(tv, obs, ex, ey, eyaw);
           },
           py::arg("target_v"), py::arg("obstacles"),
           py::arg("ego_x"), py::arg("ego_y"), py::arg("ego_yaw"))
      .def("set_params", &SpeedGovernorAdapter::set_params,
           py::arg("T_horizon"), py::arg("dt"),
           py::arg("decel_max"), py::arg("margin"));
}
