#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <vector>

#include "world/scenario.hpp"

namespace py = pybind11;
using namespace ad::world;

PYBIND11_MODULE(scenario_cpp, mod) {
    mod.doc() = "Scenario world simulation — agents, kinematics, scenario factories.";

    mod.attr("LANE") = LANE;

    py::class_<Agent>(mod, "Agent")
        .def(py::init<int, double, double, double, double, std::string>(),
             py::arg("id"), py::arg("x"), py::arg("y"), py::arg("vx"),
             py::arg("vy") = 0.0, py::arg("kind") = "car")
        .def_readwrite("id",   &Agent::id)
        .def_readwrite("x",    &Agent::x)
        .def_readwrite("y",    &Agent::y)
        .def_readwrite("vx",   &Agent::vx)
        .def_readwrite("vy",   &Agent::vy)
        .def_readwrite("kind", &Agent::kind)
        .def_readwrite("t",    &Agent::t)
        .def("set_brake",   &Agent::set_brake,   py::arg("t_start"), py::arg("decel"))
        .def("set_cut_in",  &Agent::set_cut_in,
             py::arg("t_start"), py::arg("t_end"), py::arg("y_target"))
        .def("step",  &Agent::step, py::arg("dt"))
        // .s property matches Python Agent.s = [x, y, vx, vy] for drop-in compatibility
        .def_property_readonly("s", [](const Agent& a) {
            return std::vector<double>{a.x, a.y, a.vx, a.vy};
        });

    py::class_<World>(mod, "World")
        .def(py::init<std::vector<Agent>>(), py::arg("agents"))
        .def_readwrite("agents", &World::agents)
        .def("step", &World::step, py::arg("dt"))
        // Returns list of (id, [x,y,vx,vy], kind) — same shape as Python world.py
        .def("true_objects", [](const World& w) {
            std::vector<std::tuple<int, std::vector<double>, std::string>> out;
            for (const auto& obj : w.true_objects())
                out.emplace_back(obj.id,
                                 std::vector<double>{obj.x, obj.y, obj.vx, obj.vy},
                                 obj.kind);
            return out;
        });

    // Scenario factories
    mod.def("scenario_lead_brake",    &scenario_lead_brake);
    mod.def("scenario_hard_brake",    &scenario_hard_brake);
    mod.def("scenario_cut_in",        &scenario_cut_in);
    mod.def("scenario_mixed",         &scenario_mixed);
    mod.def("scenario_acc_follow",    &scenario_acc_follow);
    mod.def("scenario_lane_avoidance",&scenario_lane_avoidance);
    mod.def("scenario_safety_stop",   &scenario_safety_stop);
}
