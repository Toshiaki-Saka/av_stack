#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <tuple>
#include "world/tracking.hpp"

namespace py = pybind11;
using namespace world;

PYBIND11_MODULE(world_cpp, mod) {
    mod.doc() = "World model: IMM Kalman multi-object tracker with prediction.";

    py::class_<MultiObjectTracker>(mod, "MultiObjectTracker")
        .def(py::init<>())
        .def_readwrite("dt", &MultiObjectTracker::dt)
        .def_readwrite("gate", &MultiObjectTracker::gate)
        .def_readwrite("min_hits", &MultiObjectTracker::min_hits)
        .def_readwrite("max_miss", &MultiObjectTracker::max_miss)
        // dets: list of (zx, zy, r00, r01, r10, r11, has_v, vx, vy)
        .def("step", [](MultiObjectTracker& mot,
                        std::vector<std::tuple<double, double, double, double, double,
                                               double, int, double, double>> dets_in) {
            std::vector<MultiObjectTracker::Det> dets;
            for (auto& d : dets_in) {
                MultiObjectTracker::Det e;
                ctrl::Mat R(2, 2);
                e.zx = std::get<0>(d); e.zy = std::get<1>(d);
                R(0, 0) = std::get<2>(d); R(0, 1) = std::get<3>(d);
                R(1, 0) = std::get<4>(d); R(1, 1) = std::get<5>(d);
                e.R = R; e.has_v = std::get<6>(d);
                e.vx = std::get<7>(d); e.vy = std::get<8>(d);
                dets.push_back(e);
            }
            mot.step(dets);
            // return confirmed tracks: (id, x, y, vx, vy, p_maneuver, confirmed)
            std::vector<std::tuple<int, double, double, double, double, double, bool>> out;
            for (auto& t : mot.tracks) {
                auto g = t.imm.estimate();
                out.emplace_back(t.id, g.x(0, 0), g.x(1, 0), g.x(2, 0), g.x(3, 0),
                                 t.imm.p_maneuver(), t.confirmed);
            }
            return out;
        }, py::arg("detections"))
        .def("predict", [](MultiObjectTracker& mot, int id, int n) {
            std::vector<std::pair<double, double>> traj;
            for (auto& t : mot.tracks) {
                if (t.id != id) continue;
                auto g = t.imm.estimate();
                double x = g.x(0, 0), y = g.x(1, 0), vx = g.x(2, 0), vy = g.x(3, 0);
                for (int k = 1; k <= n; ++k)
                    traj.emplace_back(x + vx * mot.dt * k, y + vy * mot.dt * k);
            }
            return traj;
        }, py::arg("id"), py::arg("n"),
           "Constant-velocity prediction of track `id` over n steps of dt.");
}
