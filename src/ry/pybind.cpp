#include "configuration.h"
#include "display.h"

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

      .def("display", &ry::Configuration::display)
      .def("camera", &ry::Configuration::camera, "bla", py::arg("frame")="camera", py::arg("renderInBackground") = false)
      .def("komo", &ry::Configuration::komo_IK);

  py::class_<ry::Display>(m, "Display")
      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  py::class_<ry::Camera>(m, "Camera")
      .def("set", &ry::Camera::set)
      .def("update", &ry::Camera::update);

  py::class_<ry::KOMOpy>(m, "KOMOpy")
      .def("optimize", &ry::KOMOpy::optimize2);

}
