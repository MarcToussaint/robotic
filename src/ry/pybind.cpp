#include "configuration.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

PYBIND11_MODULE(libry, m) {
  py::class_<ry::Configuration>(m, "Configuration")
      .def(py::init<>())

      .def("addFile", &ry::Configuration::addFile)
      .def("addFrame", &ry::Configuration::addFrame)

      .def("getJointNames", &ry::Configuration::getJointNames)
      .def("getJointState", &ry::Configuration::getJointState, "bla", py::arg("joints") = I_StringA())
      .def("setJointState", &ry::Configuration::setJointState)

      .def("getFrameNames", &ry::Configuration::getFrameNames)
      .def("getFrameState", &ry::Configuration::getFrameState)
      .def("setFrameState", &ry::Configuration::setFrameState)

      .def("getPairDistance", &ry::Configuration::getPairDistance)

//      .def("display", &ry::Configuration::display)
      .def("camera", &ry::Configuration::camera, "bla", py::arg("frame")="", py::arg("renderInBackground") = false)
      .def("komo", &ry::Configuration::komo_IK)
      .def("komo_CGO", &ry::Configuration::komo_CGO);

//  py::class_<ry::Display>(m, "Display")
//      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
//      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  py::class_<ry::Camera>(m, "Camera")
      .def("set", &ry::Camera::set)
      .def("update", (void (ry::Camera::*)(bool)) &ry::Camera::update)
      .def("update", (void (ry::Camera::*)(std::string, bool)) &ry::Camera::update);

  py::class_<ry::KOMOpy>(m, "KOMOpy")
      .def("makeObjectsFree", &ry::KOMOpy::makeObjectsFree)

      .def("clearObjectives", &ry::KOMOpy::clearObjectives)
      .def("addObjective", &ry::KOMOpy::addObjective, "core method to add an objective",
           py::arg("confs")=std::vector<int>(),
           py::arg("timeInterval")=std::vector<double>(),
           py::arg("type"),
           py::arg("feature"),
           py::arg("frames")=I_StringA(),
           py::arg("scale")=std::vector<double>(),
           py::arg("target")=std::vector<double>(),
           py::arg("params")=std::map<std::string, std::vector<double>>() )

      .def("addObjectives", &ry::KOMOpy::addObjectives)

      .def("add_StableRelativePose", &ry::KOMOpy::add_StableRelativePose, "", py::arg("confs"), py::arg("gripper"), py::arg("object"))
      .def("add_StablePose", &ry::KOMOpy::add_StablePose, "", py::arg("confs"), py::arg("object"))

      .def("add_grasp", &ry::KOMOpy::add_grasp)
      .def("add_place", &ry::KOMOpy::add_place)
      .def("add_resting", &ry::KOMOpy::add_resting)
      .def("add_restingRelative", &ry::KOMOpy::add_restingRelative)

      .def("optimize", &ry::KOMOpy::optimize)
      .def("getConfiguration", &ry::KOMOpy::getConfiguration)
      ;

}
