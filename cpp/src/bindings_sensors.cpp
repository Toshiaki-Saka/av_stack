#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>
#include <vector>

#include "sensors/sensors.hpp"

namespace py = pybind11;
using namespace ad::sensors;

// Convert Python world.true_objects() format → ObjectEntry list
static std::vector<ad::world::ObjectEntry>
_parse_objects(const std::vector<std::tuple<int, std::vector<double>, std::string>>& raw)
{
    std::vector<ad::world::ObjectEntry> objs;
    objs.reserve(raw.size());
    for (const auto& r : raw) {
        const auto& s = std::get<1>(r);
        objs.push_back({std::get<0>(r),
                        s.size()>0?s[0]:0.0,
                        s.size()>1?s[1]:0.0,
                        s.size()>2?s[2]:0.0,
                        s.size()>3?s[3]:0.0,
                        std::get<2>(r)});
    }
    return objs;
}

// Raw objects type alias for binding signatures
using RawObjects = std::vector<std::tuple<int, std::vector<double>, std::string>>;

// Detection as Python tuple: (z0,z1, R00,R01,R10,R11, sensor, has_vr, vr, has_kind, kind)
using DetTuple = std::tuple<double, double,
                            double, double, double, double,
                            std::string, bool, double, bool, std::string>;

static DetTuple to_tuple(const Detection& d) {
    return {d.z[0], d.z[1],
            d.R[0], d.R[1], d.R[2], d.R[3],
            d.sensor, d.has_vr, d.vr, d.has_kind, d.kind};
}

static std::vector<DetTuple>
dets_to_tuples(const std::vector<Detection>& dets) {
    std::vector<DetTuple> out;
    out.reserve(dets.size());
    for (const auto& d : dets) out.push_back(to_tuple(d));
    return out;
}

static std::array<double,3> ego_from_vec(const std::vector<double>& ego) {
    return {ego.size()>0?ego[0]:0.0,
            ego.size()>1?ego[1]:0.0,
            ego.size()>2?ego[2]:0.0};
}

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

PYBIND11_MODULE(sensors_cpp, mod) {
    mod.doc() = "Sensor models: Lidar, Radar, Camera, SensorSuite.";

    mod.def("seed_rng", &seed_rng, py::arg("seed"),
            "Seed the shared sensor RNG for reproducible simulation.");

    py::class_<Lidar>(mod, "Lidar")
        .def(py::init<double,double,double,double>(),
             py::arg("max_range")=80.0, py::arg("half_fov")=M_PI/3.0,
             py::arg("sigma")=0.15,     py::arg("p_miss")=0.05)
        .def_readwrite("max_range", &Lidar::max_range)
        .def_readwrite("sigma",     &Lidar::sigma)
        .def("measure", [](const Lidar& l,
             const std::vector<double>& ego,
             const RawObjects& objs) {
            auto e = ego_from_vec(ego);
            return dets_to_tuples(l.measure(e[0], e[1], e[2], _parse_objects(objs)));
        }, py::arg("ego"), py::arg("objects"));

    py::class_<Radar>(mod, "Radar")
        .def(py::init<double,double,double,double,double,double>(),
             py::arg("max_range")=120.0, py::arg("half_fov")=M_PI/4.0,
             py::arg("sigma_r")=0.4, py::arg("sigma_lat")=1.2,
             py::arg("sigma_vr")=0.1, py::arg("p_miss")=0.05)
        .def("measure", [](const Radar& r,
             const std::vector<double>& ego,
             const RawObjects& objs) {
            auto e = ego_from_vec(ego);
            return dets_to_tuples(r.measure(e[0], e[1], e[2], _parse_objects(objs)));
        }, py::arg("ego"), py::arg("objects"));

    py::class_<Camera>(mod, "Camera")
        .def(py::init<double,double,double,double,double>(),
             py::arg("max_range")=70.0,
             py::arg("half_fov")=35.0*M_PI/180.0,
             py::arg("sigma_bearing")=0.6*M_PI/180.0,
             py::arg("range_rel")=0.10,
             py::arg("p_miss")=0.04)
        .def("measure", [](const Camera& c,
             const std::vector<double>& ego,
             const RawObjects& objs) {
            auto e = ego_from_vec(ego);
            return dets_to_tuples(c.measure(e[0], e[1], e[2], _parse_objects(objs)));
        }, py::arg("ego"), py::arg("objects"));

    py::class_<SensorSuite>(mod, "SensorSuite")
        .def(py::init<>())
        .def_readwrite("lidar",  &SensorSuite::lidar)
        .def_readwrite("radar",  &SensorSuite::radar)
        .def_readwrite("camera", &SensorSuite::camera)
        .def("measure", [](const SensorSuite& s,
             const std::vector<double>& ego,
             const RawObjects& objs) {
            auto e = ego_from_vec(ego);
            return dets_to_tuples(s.measure(e[0], e[1], e[2], _parse_objects(objs)));
        }, py::arg("ego"), py::arg("objects"));
}
