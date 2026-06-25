#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/trajectory.hpp"
#include "planning/path.hpp"
#include "planning/planner.hpp"

namespace py = pybind11;
using namespace ad::planning;

PYBIND11_MODULE(planning_cpp, mod) {
    mod.doc() = "Planning: IDM trajectory planner, Trajectory, Path.";

    // ---- Trajectory ----
    py::class_<Trajectory>(mod, "Trajectory")
        .def(py::init<std::vector<double>, std::vector<double>,
                      std::vector<double>, double, double>(),
             py::arg("x"), py::arg("y"),
             py::arg("v")  = std::vector<double>{},
             py::arg("dt") = 0.1,
             py::arg("L")  = 2.7)
        .def_readonly("x",     &Trajectory::x)
        .def_readonly("y",     &Trajectory::y)
        .def_readonly("psi",   &Trajectory::psi)
        .def_readonly("v",     &Trajectory::v)
        .def_readonly("kappa", &Trajectory::kappa)
        .def_readonly("n",     &Trajectory::n)
        .def("nearest", &Trajectory::nearest, py::arg("px"), py::arg("py"))
        .def("errors", [](const Trajectory& t, const std::vector<double>& s) {
            double sx=s.size()>0?s[0]:0, sy=s.size()>1?s[1]:0,
                   sp=s.size()>2?s[2]:0, sv=s.size()>3?s[3]:0;
            auto r = t.errors(sx, sy, sp, sv);
            return std::vector<double>{r[0], r[1], r[2], r[3]};
        }, py::arg("state"))
        .def("mpc_window", [](const Trajectory& t,
                              const std::vector<double>& s, int N) {
            double sx=s.size()>0?s[0]:0, sy=s.size()>1?s[1]:0;
            std::vector<double> sref, uref;
            t.mpc_window(sx, sy, N, sref, uref);
            return std::make_pair(sref, uref);
        }, py::arg("state"), py::arg("N"));

    // ---- Path ----
    py::class_<Path>(mod, "Path")
        .def(py::init<double,double,double,double,double>(),
             py::arg("v_ref") = 12.0, py::arg("dt") = 0.1, py::arg("T") = 14.0,
             py::arg("L")     = 2.7,  py::arg("lane") = 3.5)
        .def_readonly("x",     &Path::x)
        .def_readonly("y",     &Path::y)
        .def_readonly("psi",   &Path::psi)
        .def_readonly("kappa", &Path::kappa)
        .def_readonly("v",     &Path::v)
        .def_readonly("n",     &Path::n)
        .def("nearest", &Path::nearest, py::arg("px"), py::arg("py"))
        .def("errors", [](const Path& p, const std::vector<double>& s) {
            double sx=s.size()>0?s[0]:0, sy=s.size()>1?s[1]:0,
                   sp=s.size()>2?s[2]:0, sv=s.size()>3?s[3]:0;
            auto r = p.errors(sx, sy, sp, sv);
            return std::vector<double>{r[0], r[1], r[2], r[3]};
        }, py::arg("state"))
        .def("mpc_window", [](const Path& p,
                              const std::vector<double>& s, int N) {
            double sx=s.size()>0?s[0]:0, sy=s.size()>1?s[1]:0;
            std::vector<double> sref, uref;
            p.mpc_window(sx, sy, N, sref, uref);
            return std::make_pair(sref, uref);
        }, py::arg("state"), py::arg("N"));

    // ---- Planner ----
    // preds_py: list of [(x0,y0), (x1,y1), ...] — same format as mot.predict() output.
    py::class_<Planner>(mod, "Planner")
        .def(py::init<double,double,double,double,double,double,
                      std::vector<double>,bool>(),
             py::arg("dt")          = 0.1,
             py::arg("horizon")     = 4.0,
             py::arg("v_des")       = 13.0,
             py::arg("L")           = 2.7,
             py::arg("safe_radius") = 4.5,
             py::arg("t_change")    = 3.0,
             py::arg("lanes")       = std::vector<double>{0.0, 3.5},
             py::arg("fault")       = false)
        .def_readwrite("v_des",       &Planner::v_des)
        .def_readwrite("safe_radius", &Planner::safe_radius)
        .def_readwrite("M",           &Planner::M)
        .def_readwrite("fault",       &Planner::fault)
        .def("plan", [](Planner& pl,
                        const std::vector<double>& ego,
                        const std::vector<std::vector<std::pair<double,double>>>& preds_py) {
            // Convert list of [(x,y)] lists to PredictedTrack vector
            std::vector<PredictedTrack> preds;
            preds.reserve(preds_py.size());
            for (const auto& pos_list : preds_py)
                preds.push_back({pos_list});
            double e[4] = {ego.size()>0?ego[0]:0, ego.size()>1?ego[1]:0,
                           ego.size()>2?ego[2]:0, ego.size()>3?ego[3]:0};
            auto result = pl.plan(e, preds);
            return std::make_pair(result.traj, result.behavior);
        }, py::arg("ego"), py::arg("preds"));
}
