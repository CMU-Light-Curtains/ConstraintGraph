#ifndef PY_HPP
#define PY_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <v1.h>
#include <v2.h>

namespace py = pybind11;
using namespace planner;

PYBIND11_MODULE(planner_py, m) {

    py::class_<CameraParameters, std::shared_ptr<CameraParameters>>(m, "CameraParameters")
            .def(py::init<>())
            .def_readwrite("width", &CameraParameters::width)
            .def_readwrite("height", &CameraParameters::height)
            .def_readwrite("fps", &CameraParameters::fps)
            .def_readwrite("cam_matrix", &CameraParameters::cam_matrix)
            .def_readwrite("cam_to_laser", &CameraParameters::cam_to_laser)
            ;

    py::class_<LaserParameters, std::shared_ptr<LaserParameters>>(m, "LaserParameters")
            .def(py::init<>())
            .def_readwrite("fov", &LaserParameters::fov)
            .def_readwrite("thickness", &LaserParameters::thickness)
            .def_readwrite("divergence", &LaserParameters::divergence)
            .def_readwrite("max_omega", &LaserParameters::max_omega)
            .def_readwrite("max_alpha", &LaserParameters::max_alpha)
            ;

    py::class_<Location, std::shared_ptr<Location>>(m, "Location")
            .def_readonly("x", &Location::x)
            .def_readonly("z", &Location::z)
            .def_readonly("r", &Location::r)
            .def_readonly("theta_cam", &Location::theta_cam)
            .def_readonly("theta_las", &Location::theta)
            .def_readonly("ki", &Location::ki)
            .def_readonly("kj", &Location::kj)
            ;

    py::class_<Interpolator, std::shared_ptr<Interpolator>>(m, "Interpolator");

    py::class_<CartesianNNInterpolator, Interpolator, std::shared_ptr<CartesianNNInterpolator>>(m, "CartesianNNInterpolator")
            .def(py::init<int, int, float, float, float, float> ())
            ;

    py::class_<PolarIdentityInterpolator, Interpolator, std::shared_ptr<PolarIdentityInterpolator>>(m, "PolarIdentityInterpolator")
            .def(py::init<int, int> ())
            ;

    py::class_<Planner, std::shared_ptr<Planner>>(m, "Planner");

    py::class_<v1::PlannerV1, std::shared_ptr<v1::PlannerV1>>(m, "PlannerV1")
            .def(py::init<const CameraParameters&,
                          const LaserParameters&,
                          const std::vector<float>&,
                          const Interpolator&,
                          bool> ())
            .def("getThetas", &v1::PlannerV1::getThetas)
            .def("getLayoutForVis", &v1::PlannerV1::getVectorizedLayout)
            .def("loadLineSurface", &v1::PlannerV1::loadLineSurface)
            .def("computeVisibleRanges", &v1::PlannerV1::computeVisibleRanges)
            .def("visiblePoints", &v1::PlannerV1::visiblePoints)
            .def("computeLayoutIntensities", &v1::PlannerV1::computeLayoutIntensities)
            .def("layoutIntensities", &v1::PlannerV1::layoutIntensities)
            .def("optGlobalCostDiscrete", &v1::PlannerV1::optGlobalCostDiscrete)
            .def("randomCurtainDiscrete", &v1::PlannerV1::randomCurtainDiscrete)
            .def("randomCurtainContinuous", &v1::PlannerV1::randomCurtainContinuous)
            .def("randomCurtainHitProb", &v1::PlannerV1::randomCurtainHitProb)
            ;

    py::class_<v2::PlannerV2, std::shared_ptr<v2::PlannerV2>>(m, "PlannerV2")
            .def(py::init<const CameraParameters&,
                          const LaserParameters&,
                          const std::vector<float>&,
                          const Interpolator&,
                          bool> ())
            .def("getThetas", &v2::PlannerV2::getThetas)
            .def("getLayoutForVis", &v2::PlannerV2::getVectorizedLayout)
            .def("loadLineSurface", &v2::PlannerV2::loadLineSurface)
            .def("computeVisibleRanges", &v2::PlannerV2::computeVisibleRanges)
            .def("visiblePoints", &v2::PlannerV2::visiblePoints)
            .def("computeLayoutIntensities", &v2::PlannerV2::computeLayoutIntensities)
            .def("layoutIntensities", &v2::PlannerV2::layoutIntensities)
            .def("optGlobalCostDiscrete", &v2::PlannerV2::optGlobalCostDiscrete)
            .def("randomCurtainDiscrete", &v2::PlannerV2::randomCurtainDiscrete)
            .def("randomCurtainContinuous", &v2::PlannerV2::randomCurtainContinuous)
            .def("randomCurtainHitProb", &v2::PlannerV2::randomCurtainHitProb)
            ;

    #ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
    #else
    m.attr("__version__") = "dev";
    #endif
}

#endif