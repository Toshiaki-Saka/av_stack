#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <array>
#include "control/bicycle.hpp"
#include "control/lqr.hpp"
#include "control/mpc.hpp"

namespace py = pybind11;
using namespace ctrl;

PYBIND11_MODULE(control_cpp, mod) {
    mod.doc() = "Vehicle control core: kinematic bicycle, PID, LQR, and constrained MPC.";

    py::class_<Bicycle>(mod, "Bicycle")
        .def(py::init<>())
        .def_readwrite("L", &Bicycle::L)
        .def("step", [](const Bicycle& b, std::vector<double> s, double a, double delta, double dt) {
            Bicycle::State st = {s[0], s[1], s[2], s[3]};
            auto n = b.step(st, {a, delta}, dt);
            return std::vector<double>{n[0], n[1], n[2], n[3]};
        }, py::arg("state"), py::arg("a"), py::arg("delta"), py::arg("dt"));

    py::class_<PID>(mod, "PID")
        .def(py::init<>())
        .def_readwrite("kp", &PID::kp).def_readwrite("ki", &PID::ki).def_readwrite("kd", &PID::kd)
        .def_readwrite("out_min", &PID::out_min).def_readwrite("out_max", &PID::out_max)
        .def_readwrite("i_min", &PID::i_min).def_readwrite("i_max", &PID::i_max)
        .def("step", &PID::step, py::arg("err"), py::arg("dt"))
        .def("reset", &PID::reset);

    py::class_<LateralLQR>(mod, "LateralLQR")
        .def(py::init<>())
        .def_readwrite("L", &LateralLQR::L).def_readwrite("dt", &LateralLQR::dt)
        .def_readwrite("q_ey", &LateralLQR::q_ey).def_readwrite("q_epsi", &LateralLQR::q_epsi)
        .def_readwrite("r_delta", &LateralLQR::r_delta)
        .def("steer", &LateralLQR::steer,
             py::arg("e_y"), py::arg("e_psi"), py::arg("v"), py::arg("kappa"));

    py::class_<MPC>(mod, "MPC")
        .def(py::init<>())
        .def_readwrite("N", &MPC::N).def_readwrite("dt", &MPC::dt)
        .def_readwrite("q_x", &MPC::q_x).def_readwrite("q_y", &MPC::q_y)
        .def_readwrite("q_psi", &MPC::q_psi).def_readwrite("q_v", &MPC::q_v)
        .def_readwrite("r_a", &MPC::r_a).def_readwrite("r_delta", &MPC::r_delta)
        .def_readwrite("a_min", &MPC::a_min).def_readwrite("a_max", &MPC::a_max)
        .def_readwrite("delta_min", &MPC::delta_min).def_readwrite("delta_max", &MPC::delta_max)
        .def_readwrite("iters", &MPC::iters)
        .def_property("L", [](const MPC& m) { return m.model.L; },
                           [](MPC& m, double L) { m.model.L = L; })
        .def("solve", [](const MPC& m, std::vector<double> s0,
                         std::vector<double> sref, std::vector<double> uref) {
            Bicycle::State st = {s0[0], s0[1], s0[2], s0[3]};
            auto u = m.solve(st, sref, uref);
            return std::vector<double>{u[0], u[1]};
        }, py::arg("state"), py::arg("sref"), py::arg("uref"),
           "Return [a, delta] tracking the reference (sref:(N+1)*4, uref:N*2).");
}
