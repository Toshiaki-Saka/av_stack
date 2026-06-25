// pybind11 adapter for ad::localization::EkfLocalizer (ad-pipeline-demo).
//
// Python interface (unchanged from previous standalone impl):
//   ekf = EkfLocalizer()
//   ekf.predict(v, steering_angle, dt)
//   ekf.update(meas_x, meas_y)
//   x, y, yaw = ekf.localize()
//   ekf.reset(x=0, y=0, yaw=0)
//
// The adapter bridges the difference in velocity semantics:
//   ad:: EKF  — velocity is a state variable updated via acceleration command.
//   av_stack  — velocity is a direct per-step input (tachometer reading).
// Bridge: acceleration = (v_desired - v_prev) / dt so that after predict()
// state_.velocity == v_desired, while the kinematic step uses v_prev.
#include <memory>
#include <tuple>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// ad-pipeline-demo headers (header-only EKF)
#include "ad/localization/ekf_localizer.hpp"

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Thin adapter: exposes the Python-friendly (v, steer, dt) interface
// ---------------------------------------------------------------------------
class EkfLocalizerAdapter {
 public:
  using Params = ad::localization::EkfLocalizer::Params;

  explicit EkfLocalizerAdapter(Params p = {}) : params_{p} { make_ekf(0, 0, 0, 0); }

  void predict(double v, double steering_angle, double dt) {
    const double v_prev = ekf_->localize().velocity;
    const double a = (dt > 1e-9) ? (v - v_prev) / dt : 0.0;
    ekf_->predict({a, steering_angle}, dt);
  }

  void update(double meas_x, double meas_y) {
    ekf_->update({meas_x, meas_y});
  }

  std::tuple<double, double, double> localize() const {
    const auto s = ekf_->localize();
    return {s.pose.x, s.pose.y, s.pose.yaw};
  }

  void reset(double x = 0.0, double y = 0.0, double yaw = 0.0) {
    make_ekf(x, y, yaw, 0.0);
  }

 private:
  void make_ekf(double x, double y, double yaw, double v) {
    ad::common::VehicleState init;
    init.pose.x = x; init.pose.y = y; init.pose.yaw = yaw;
    init.velocity = v;
    ekf_ = std::make_unique<ad::localization::EkfLocalizer>(init, params_);
  }

  Params params_;
  std::unique_ptr<ad::localization::EkfLocalizer> ekf_;
};

// ---------------------------------------------------------------------------
// pybind11 module
// ---------------------------------------------------------------------------
PYBIND11_MODULE(localization_cpp, mod) {
  mod.doc() = "EKF pose localizer — wraps ad::localization::EkfLocalizer "
              "(ad-pipeline-demo shared implementation).";

  py::class_<EkfLocalizerAdapter::Params>(mod, "EkfParams")
      .def(py::init<>())
      .def_readwrite("wheelbase",         &EkfLocalizerAdapter::Params::wheelbase)
      .def_readwrite("process_pos",       &EkfLocalizerAdapter::Params::process_pos)
      .def_readwrite("process_yaw",       &EkfLocalizerAdapter::Params::process_yaw)
      .def_readwrite("meas_pos",          &EkfLocalizerAdapter::Params::meas_pos);

  py::class_<EkfLocalizerAdapter>(mod, "EkfLocalizer")
      .def(py::init<>())
      .def(py::init<EkfLocalizerAdapter::Params>(), py::arg("params"))
      .def("predict", &EkfLocalizerAdapter::predict,
           py::arg("v"), py::arg("steering_angle"), py::arg("dt"))
      .def("update",  &EkfLocalizerAdapter::update,
           py::arg("meas_x"), py::arg("meas_y"))
      .def("localize", &EkfLocalizerAdapter::localize,
           "Returns (x, y, yaw) tuple.")
      .def("reset", &EkfLocalizerAdapter::reset,
           py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0);
}
