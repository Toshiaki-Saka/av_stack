#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <vector>

#include "perception/fusion.hpp"

namespace py = pybind11;
using namespace ad::perception;

// Convert the sensor binding tuple format to ad::sensors::Detection
// Tuple: (z0, z1, R00, R01, R10, R11, sensor, has_vr, vr, has_kind, kind)
using RawDet = std::tuple<double, double,
                          double, double, double, double,
                          std::string, bool, double, bool, std::string>;

static ad::sensors::Detection det_from_tuple(const RawDet& t) {
    ad::sensors::Detection d;
    d.z[0] = std::get<0>(t); d.z[1] = std::get<1>(t);
    d.R[0] = std::get<2>(t); d.R[1] = std::get<3>(t);
    d.R[2] = std::get<4>(t); d.R[3] = std::get<5>(t);
    d.sensor   = std::get<6>(t);
    d.has_vr   = std::get<7>(t);
    d.vr       = std::get<8>(t);
    d.has_kind = std::get<9>(t);
    d.kind     = std::get<10>(t);
    return d;
}

// FusedDetection as Python tuple:
// (z0, z1, R00, R01, R10, R11, kind, has_v, vx, vy, sensors)
using FusedTuple = std::tuple<double, double,
                              double, double, double, double,
                              std::string, bool, double, double,
                              std::vector<std::string>>;

static FusedTuple fd_to_tuple(const FusedDetection& fd) {
    return {fd.z[0], fd.z[1],
            fd.R[0], fd.R[1], fd.R[2], fd.R[3],
            fd.kind, fd.has_v, fd.vx, fd.vy, fd.sensors};
}

PYBIND11_MODULE(perception_cpp, mod) {
    mod.doc() = "Sensor fusion: Mahalanobis clustering + information-filter fusion.";

    // fuse(raw_detections, gate=9.21) → list of fused detection tuples
    mod.def("fuse", [](const std::vector<RawDet>& raw, double gate) {
        std::vector<ad::sensors::Detection> dets;
        dets.reserve(raw.size());
        for (const auto& r : raw) dets.push_back(det_from_tuple(r));
        auto result = fuse(dets, gate);
        std::vector<FusedTuple> out;
        for (const auto& fd : result) out.push_back(fd_to_tuple(fd));
        return out;
    }, py::arg("detections"), py::arg("gate") = 9.21,
    "Cluster and fuse raw sensor detections via information filter.");

    // perceive(raw_detections, gate) — sugar: fuse a flat detection tuple list.
    // Call suite.measure(ego, objects) first to get raw_detections.
    mod.def("perceive", [](const std::vector<RawDet>& raw, double gate) {
        std::vector<ad::sensors::Detection> dets;
        dets.reserve(raw.size());
        for (const auto& r : raw) dets.push_back(det_from_tuple(r));
        auto result = fuse(dets, gate);
        std::vector<FusedTuple> out;
        for (const auto& fd : result) out.push_back(fd_to_tuple(fd));
        return out;
    }, py::arg("detections"), py::arg("gate") = 9.21,
    "Cluster and fuse raw sensor detections; equivalent to fuse(detections).");
}
