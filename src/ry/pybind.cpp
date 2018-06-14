#include "configuration.h"
#include "display.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

PYBIND11_MODULE(libry, m) {
  py::class_<ry::Kin>(m, "Kin")
      .def(py::init<>())
      .def("addFile", &ry::Kin::addFile)
      .def("getJointState", &ry::Kin::getJointState)
      .def("display", &ry::Kin::display);

  py::class_<ry::KinDisplay>(m, "KinDisplay")
      .def("update", &ry::KinDisplay::update);
}
