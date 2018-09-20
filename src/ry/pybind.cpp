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

  py::class_<ry::FrameNames>(m, "FrameNames", py::dynamic_attr())
      .def(py::init<>());

  //===========================================================================

  py::class_<ry::Configuration>(m, "Configuration")
      .def(py::init<>())

  .def_readwrite("frameNames", &ry::Configuration::frameNames)

  .def("clear", [](ry::Configuration& self) {
    self.K.set()->clear();
  } )

  .def("addFile", [&m](ry::Configuration& self, const std::string& file) {
    self.K.set()->addFile(file.c_str());
    auto lock=self.K.get();
//    this->attr("frameNames").attr("HOLLA") = 1;
//    for(rai::Frame *f : self.K().frames) self.framenames[f->name.p] = f->ID;
  } )

  .def("addFrame", [](ry::Configuration& self, const std::string& name, const std::string& parent, const std::string& args) {
    return self.K.set()->addFrame(name.c_str(), parent.c_str(), args.c_str())->ID;
  } )

  .def("delFrame", [](ry::Configuration& self, const std::string& name) {
    auto Kset = self.K.set();
    rai::Frame *p = Kset->getFrameByName(name.c_str(), true);
    if(p) delete p;
  } )

  .def("getJointNames", [](ry::Configuration& self) {
    return I_conv(self.K.get()->getJointNames());
  } )

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

  .def("evalFeature", [](ry::Configuration& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    auto Kget = self.K.get();
    arr y,J;
    Kget->evalFeature(y, J, fs, I_conv(frames));
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  } )

      .def("getPairDistance", &ry::Configuration::getPairDistance)

      .def("useJointGroups", &ry::Configuration::useJointGroups)
      .def("setActiveJoints", &ry::Configuration::setActiveJoints)
      .def("makeObjectsFree", &ry::Configuration::makeObjectsFree)

      .def("camera", &ry::Configuration::camera, "bla", py::arg("frame")="", py::arg("renderInBackground") = false)
      .def("komo_IK", &ry::Configuration::komo_IK)
      .def("komo_path", &ry::Configuration::komo_path, "",
           py::arg("phases"),
           py::arg("stepsPerPhase")=20,
           py::arg("timePerPhase")=5. )
      .def("komo_CGO", &ry::Configuration::komo_CGO)
      .def("lgp", &ry::Configuration::lgp);

//  py::class_<ry::Display>(m, "Display")
//      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
//      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  //===========================================================================

  py::class_<ry::Camera>(m, "Camera")
      .def("set", &ry::Camera::set)
      .def("update", (void (ry::Camera::*)(bool)) &ry::Camera::update,
           py::arg("wait")=false)
      .def("update", (void (ry::Camera::*)(std::string, bool)) &ry::Camera::update);

  //===========================================================================

  py::class_<ry::KOMOpy>(m, "KOMOpy")
      .def("makeObjectsFree", &ry::KOMOpy::makeObjectsFree)
      .def("activateCollisionPairs", &ry::KOMOpy::activateCollisionPairs)
      .def("deactivateCollisionPairs", &ry::KOMOpy::deactivateCollisionPairs)
      .def("timeOptimization", &ry::KOMOpy::timeOptimization)

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

      .def("addSkeleton", &ry::KOMOpy::addSkeleton)
      .def("setSkeleton", &ry::KOMOpy::setSkeleton)
      .def("skeleton2bound", &ry::KOMOpy::skeleton2bound)


      .def("optimize", &ry::KOMOpy::optimize)
      .def("getT", &ry::KOMOpy::getT)
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

  //===========================================================================

#define ENUMVAL(x) .value(#x, x)
  py::enum_<FeatureSymbol>(m, "FeatureSymbol")
      ENUMVAL(FS_none)
      ENUMVAL(FS_position)
      ENUMVAL(FS_positionDiff)
      ENUMVAL(FS_positionRel)
      ENUMVAL(FS_quaternion)
      ENUMVAL(FS_quaternionDiff)
      ENUMVAL(FS_quaternionRel)
      ENUMVAL(FS_pose)
      ENUMVAL(FS_poseDiff)
      ENUMVAL(FS_poseRel)
      ENUMVAL(FS_vectorX)
      ENUMVAL(FS_vectorXDiff)
      ENUMVAL(FS_vectorXRel)
      ENUMVAL(FS_vectorY)
      ENUMVAL(FS_vectorYDiff)
      ENUMVAL(FS_vectorYRel)
      ENUMVAL(FS_vectorZ)
      ENUMVAL(FS_vectorZDiff)
      ENUMVAL(FS_vectorZRel)
      ENUMVAL(FS_scalarProductXX)
      ENUMVAL(FS_scalarProductXY)
      ENUMVAL(FS_scalarProductXZ)
      ENUMVAL(FS_scalarProductYX)
      ENUMVAL(FS_scalarProductYY)
      ENUMVAL(FS_scalarProductYZ)
      ENUMVAL(FS_scalarProductZZ)
      ENUMVAL(FS_gazeAt)

      ENUMVAL(FS_accumulatedCollisions)
      ENUMVAL(FS_jointLimits)
      ENUMVAL(FS_distance)

      ENUMVAL(FS_qItself)

      ENUMVAL(FS_aboveBox)
      ENUMVAL(FS_insideBox)

      ENUMVAL(FS_standingAbove)

      ENUMVAL(FS_physics)
      ENUMVAL(FS_contactConstraints)
      ENUMVAL(FS_energy)

      .export_values();
#undef ENUMVAL

}
