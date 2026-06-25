#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <vector>

#include "safety/guardrail.hpp"

namespace py = pybind11;
using namespace ad::safety;

// Track tuple from world_cpp: (id, x, y, vx, vy, p_man, confirmed)
using TrackTuple = std::tuple<int, double, double, double, double, double, bool>;

static TrackEntry track_from_tuple(const TrackTuple& t) {
    return {std::get<0>(t),
            std::get<1>(t), std::get<2>(t),
            std::get<3>(t), std::get<4>(t),
            std::get<6>(t)};
}

PYBIND11_MODULE(guardrail_cpp, mod) {
    mod.doc() = "RSS safety guardrail — longitudinal/lateral safety + latch override.";

    py::class_<Guardrail>(mod, "Guardrail")
        .def(py::init<double,double,double,double,double,double,int,bool>(),
             py::arg("rho")          = 0.4,
             py::arg("a_accel_max")  = 1.0,
             py::arg("b_min")        = 4.0,
             py::arg("b_max_lead")   = 8.0,
             py::arg("b_emergency")  = 6.0,
             py::arg("ttc_min")      = 2.5,
             py::arg("hold")         = 15,
             py::arg("use_lateral")  = true)
        .def_readwrite("rho",          &Guardrail::rho)
        .def_readwrite("ttc_min",     &Guardrail::ttc_min)
        .def_readwrite("b_emergency", &Guardrail::b_emergency)
        .def_readwrite("use_lateral", &Guardrail::use_lateral)
        .def("rss_min_distance",
             &Guardrail::rss_min_distance,
             py::arg("v_ego"), py::arg("v_lead"))
        .def("rss_lateral_min_distance",
             &Guardrail::rss_lateral_min_distance,
             py::arg("v_other_lat"))
        // ego = [x,y,vx,vy], cmd = (a, delta), tracks from world_cpp.step()
        .def("check", [](Guardrail& g,
                         const std::vector<double>& ego,
                         const std::vector<double>& cmd,  // [a, delta]
                         const std::vector<TrackTuple>& tracks_py) {
            double e[4] = {ego.size()>0?ego[0]:0, ego.size()>1?ego[1]:0,
                           ego.size()>2?ego[2]:0, ego.size()>3?ego[3]:0};
            double c[2] = {cmd.size()>0?cmd[0]:0, cmd.size()>1?cmd[1]:0};
            std::vector<TrackEntry> tracks;
            tracks.reserve(tracks_py.size());
            for (const auto& t : tracks_py) tracks.push_back(track_from_tuple(t));
            auto r = g.check(e, c, tracks);
            return std::make_tuple(r.a, r.delta, r.status, r.reason);
        }, py::arg("ego"), py::arg("cmd"), py::arg("tracks"),
        "Returns (a, delta, status, reason). status is 'OK' or 'OVERRIDE'.");
}
