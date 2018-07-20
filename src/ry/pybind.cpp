#include "configuration.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;


py::dict graph2dict(const Graph& G){
  py::dict dict;
  for(Node *n:G){
    rai::String key;
    if(n->keys.N) key=n->keys.last();
    else key <<n->index;

    //-- write value
    if(n->isGraph()) {
      dict[key.p] = graph2dict(n->get<Graph>());
    } else if(n->isOfType<rai::String>()) {
      dict[key.p] = n->get<rai::String>().p;
    } else if(n->isOfType<arr>()) {
      dict[key.p] = conv_arr2stdvec( n->get<arr>() );
    } else if(n->isOfType<double>()) {
      dict[key.p] = n->get<double>();
    } else if(n->isOfType<int>()) {
      dict[key.p] = n->get<int>();
    } else if(n->isOfType<uint>()) {
      dict[key.p] = n->get<uint>();
    } else if(n->isOfType<bool>()) {
      dict[key.p] = n->get<bool>();
    } else {
    }

  }
  return dict;
}

py::list graph2list(const Graph& G){
  py::list list;
  for(Node *n:G){
    //-- write value
    if(n->isGraph()) {
      list.append( graph2dict(n->get<Graph>()) );
    } else if(n->isOfType<rai::String>()) {
      list.append( n->get<rai::String>().p );
    } else if(n->isOfType<arr>()) {
      list.append( conv_arr2stdvec( n->get<arr>() ) );
    } else if(n->isOfType<double>()) {
      list.append( n->get<double>() );
    } else if(n->isOfType<int>()) {
      list.append( n->get<int>() );
    } else if(n->isOfType<uint>()) {
      list.append( n->get<uint>() );
    } else if(n->isOfType<bool>()) {
      list.append( n->get<bool>() );
    } else {
    }

  }
  return list;
}

PYBIND11_MODULE(libry, m) {
  py::class_<ry::Configuration>(m, "Configuration")
      .def(py::init<>())

      .def("clear", &ry::Configuration::clear)
      .def("addFile", &ry::Configuration::addFile)
      .def("addFrame", &ry::Configuration::addFrame)
      .def("delFrame", &ry::Configuration::delFrame)

      .def("getJointNames", &ry::Configuration::getJointNames)
      .def("getJointState", &ry::Configuration::getJointState, "",
           py::arg("joints") = ry::I_StringA())
      .def("setJointState", &ry::Configuration::setJointState, "",
           py::arg("q"),
           py::arg("joints") = ry::I_StringA() )

      .def("getFrameNames", &ry::Configuration::getFrameNames)
      .def("getFrameState", (pybind11::array (ry::Configuration::*)()) &ry::Configuration::getFrameState)
      .def("getFrameState", (pybind11::array (ry::Configuration::*)(const char*)) &ry::Configuration::getFrameState)
      .def("setFrameState", &ry::Configuration::setFrameState, "",
           py::arg("X"),
           py::arg("frames") = ry::I_StringA(),
           py::arg("calc_q_from_X") = true )

      .def("getPairDistance", &ry::Configuration::getPairDistance)

      .def("useJointGroups", &ry::Configuration::useJointGroups)
      .def("setActiveJoints", &ry::Configuration::setActiveJoints)
      .def("makeObjectsFree", &ry::Configuration::makeObjectsFree)

      .def("camera", &ry::Configuration::camera, "bla", py::arg("frame")="", py::arg("renderInBackground") = false)
      .def("komo_IK", &ry::Configuration::komo_IK)
      .def("komo_path", &ry::Configuration::komo_path)
      .def("komo_CGO", &ry::Configuration::komo_CGO)
      .def("lgp", &ry::Configuration::lgp);

//  py::class_<ry::Display>(m, "Display")
//      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
//      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  py::class_<ry::Camera>(m, "Camera")
      .def("set", &ry::Camera::set)
      .def("update", (void (ry::Camera::*)(bool)) &ry::Camera::update,
           py::arg("wait")=false)
      .def("update", (void (ry::Camera::*)(std::string, bool)) &ry::Camera::update);

  py::class_<ry::KOMOpy>(m, "KOMOpy")
      .def("makeObjectsFree", &ry::KOMOpy::makeObjectsFree)

      .def("clearObjectives", &ry::KOMOpy::clearObjectives)
      .def("addObjective", &ry::KOMOpy::addObjective, "core method to add an objective",
           py::arg("confs")=std::vector<int>(),
           py::arg("timeInterval")=std::vector<double>(),
           py::arg("type"),
           py::arg("feature"),
           py::arg("frames")=ry::I_StringA(),
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
      .def("getReport",
           [](ry::KOMOpy& self) -> py::list{
             Graph G = self.getProblemGraph();
             return graph2list(G);
           } )
      .def("getConstraintViolations", &ry::KOMOpy::getConstraintViolations)
      .def("getCosts", &ry::KOMOpy::getCosts)
      ;

  py::class_<ry::LGPpy>(m, "LGPpy")
      .def("optimizeFixedSequence", &ry::LGPpy::optimizeFixedSequence)
      ;
}
