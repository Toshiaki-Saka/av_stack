#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <vector>

#include "world/occupancy.hpp"

namespace py = pybind11;
using namespace ad::world;

// preds_py: list of per-track predicted positions, each a list of (x,y) pairs
//   [ [(x0,y0),(x1,y1),...], ... ]
static std::vector<std::vector<PredPos>>
parse_preds(const std::vector<std::vector<std::pair<double,double>>>& raw)
{
    std::vector<std::vector<PredPos>> result;
    result.reserve(raw.size());
    for (const auto& track : raw) {
        std::vector<PredPos> ps;
        ps.reserve(track.size());
        for (const auto& xy : track)
            ps.push_back({xy.first, xy.second});
        result.push_back(std::move(ps));
    }
    return result;
}

PYBIND11_MODULE(occupancy_cpp, mod) {
    mod.doc() = "Probabilistic 2-D occupancy grid via Gaussian splatting.";

    py::class_<OccupancyGrid>(mod, "OccupancyGrid")
        .def(py::init<double,double,double,double,double,double,double,double,double>(),
             py::arg("x_min")   = 0.0,
             py::arg("x_range") = 80.0,
             py::arg("y_min")   = -2.0,
             py::arg("y_max")   = 5.5,
             py::arg("res")     = 0.5,
             py::arg("sigma0")  = 0.6,
             py::arg("growth")  = 0.06,
             py::arg("car_l")   = 2.2,
             py::arg("car_w")   = 0.9)
        .def_readonly("nx",  &OccupancyGrid::nx)
        .def_readonly("ny",  &OccupancyGrid::ny)
        .def_readonly("res", &OccupancyGrid::res)
        // predict(preds, horizons) → list of flat float grids
        .def("predict", [](const OccupancyGrid& og,
             const std::vector<std::vector<std::pair<double,double>>>& preds_py,
             const std::vector<int>& horizons) {
            return og.predict(parse_preds(preds_py), horizons);
        }, py::arg("preds"), py::arg("horizons"))
        .def("sample", [](const OccupancyGrid& og,
             const std::vector<float>& grid,
             double wx, double wy) {
            return og.sample(grid, wx, wy);
        }, py::arg("grid"), py::arg("x"), py::arg("y"))
        .def("path_risk", [](const OccupancyGrid& og,
             const std::vector<float>& grid,
             const std::vector<double>& xs,
             const std::vector<double>& ys) {
            return og.path_risk(grid, xs, ys);
        }, py::arg("grid"), py::arg("xs"), py::arg("ys"));
}
