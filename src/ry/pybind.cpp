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
      .def("getJointState", &ry::Configuration::getJointState)
      .def("display", &ry::Configuration::display);

  py::class_<ry::Display>(m, "Display")
      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);
}
