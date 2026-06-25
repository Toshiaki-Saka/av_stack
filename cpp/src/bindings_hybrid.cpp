#include <pybind11/pybind11.h>
#include <tuple>

#include "hybrid/arbiter.hpp"
#include "hybrid/confidence.hpp"

namespace py = pybind11;
using namespace hybrid;

PYBIND11_MODULE(hybrid_cpp, mod) {
    mod.doc() = "Confidence-weighted NN/MPC arbiter for hybrid self-driving stacks.";

    py::enum_<Source>(mod, "Source")
        .value("NN",       Source::NN)
        .value("MPC",      Source::MPC)
        .value("BLEND",    Source::BLEND)
        .value("FAILSAFE", Source::FAILSAFE)
        .export_values();

    py::class_<ArbiterResult>(mod, "ArbiterResult")
        .def_readonly("steer",  &ArbiterResult::steer)
        .def_readonly("source", &ArbiterResult::source)
        .def("__repr__", [](const ArbiterResult& r) {
            const char* s = "FAILSAFE";
            if (r.source == Source::NN)    s = "NN";
            if (r.source == Source::MPC)   s = "MPC";
            if (r.source == Source::BLEND) s = "BLEND";
            return std::string("ArbiterResult(steer=") + std::to_string(r.steer)
                   + ", source=" + s + ")";
        });

    py::class_<Arbiter>(mod, "Arbiter")
        .def(py::init<>())
        .def_readwrite("thresh_failsafe", &Arbiter::thresh_failsafe)
        .def_readwrite("thresh_dominant", &Arbiter::thresh_dominant)
        .def("arbitrate", &Arbiter::arbitrate,
             py::arg("steer_nn"),  py::arg("conf_nn"),
             py::arg("steer_mpc"), py::arg("conf_mpc"),
             "Returns ArbiterResult(steer, source).");

    // Free functions
    mod.def("confidence_modular", &confidence_modular,
            py::arg("tracking_residual"), py::arg("steer"), py::arg("max_steer") = 0.6,
            "Modular-channel confidence in [0,1].");
    mod.def("confidence_e2e", &confidence_e2e,
            py::arg("epistemic_std"), py::arg("ood_distance"),
            "E2E-channel confidence in [0,1].");
}
