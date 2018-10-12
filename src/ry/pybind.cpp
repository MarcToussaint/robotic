#include <Kin/kin.h>
#include <Core/graph.h>
#include <Core/thread.h>
#include <Kin/frame.h>
#include <Kin/kinViewer.h>
#include "types.h"
#include "komo-py.h"
#include "lgp-py.h"

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

py::list uintA2tuple(const uintA& tup){
  py::tuple tuple;
  for(uint i=0;i<tup.N;i++) tuple[i] = tup(i);
  return tuple;
}

#define METHOD_set(method) .def(#method, [](ry::Config& self) { self.set()->method(); } )
#define METHOD_set1(method, arg1) .def(#method, [](ry::Config& self) { self.set()->method(arg1); } )

namespace ry{

  typedef Var<rai::KinematicWorld> Config;

  struct ConfigView { ptr<KinViewer> view; };

  struct RyFeature { Feature *feature=0; };
}

PYBIND11_MODULE(libry, m) {

  //===========================================================================

  py::class_<ry::Config>(m, "Config")
      .def(py::init<>())

  METHOD_set(clear)

  .def("copy", [](ry::Config& self, ry::Config& K2) {
    self.set()->copy(K2.get());
  } )

  .def("addFile", [](ry::Config& self, const std::string& file) {
    self.set()->addFile(file.c_str());
  } )

  .def("addFrame", [](ry::Config& self, const std::string& name, const std::string& parent, const std::string& args) {
    return self.set()->addFrame(name.c_str(), parent.c_str(), args.c_str())->ID;
  } )

  .def("delFrame", [](ry::Config& self, const std::string& name) {
    auto Kset = self.set();
    rai::Frame *p = Kset->getFrameByName(name.c_str(), true);
    if(p) delete p;
  } )

  .def("addObject", [](ry::Config& self, const std::string& name, const std::string& parent,
       rai::ShapeType shape,
       const std::vector<double>& size,
       const std::vector<double>& color,
       const std::vector<double>& pos,
       const std::vector<double>& quat,
       const std::vector<double>& rot,
       double radius){
    auto Kset = self.set();
    rai::Frame *f = Kset->addObject(shape, conv_stdvec2arr(size), conv_stdvec2arr(color), radius);
    f->name = name;
    if(parent.size()){
      rai::Frame *p = Kset->getFrameByName(parent.c_str());
      if(p) f->linkFrom(p);
    }
    if(pos.size()) f->Q.pos.set(pos);
    if(quat.size()) f->Q.rot.set(quat);
    if(rot.size()) f->Q.addRelativeRotationDeg(rot[0], rot[1], rot[2], rot[3]);
    if(f->parent){
      f->X = f->parent->X * f->Q;
    }else{
      f->X = f->Q;
    }
  }, "",
    py::arg("name"),
    py::arg("parent") = std::string(),
    py::arg("shape"),
    py::arg("size") = std::vector<double>(),
    py::arg("color") = std::vector<double>(),
    py::arg("pos") = std::vector<double>(),
    py::arg("quat") = std::vector<double>(),
    py::arg("rot") = std::vector<double>(),
    py::arg("radius") = -1. )


  .def("getJointNames", [](ry::Config& self) {
    return I_conv(self.get()->getJointNames());
  } )

  .def("getJointDimension", [](ry::Config& self) {
    return self.get()->getJointStateDimension();
  } )

  .def("getJointState", [](ry::Config& self, const ry::I_StringA& joints) {
    arr q;
    if(joints.size()) q = self.get()->getJointState(I_conv(joints));
    else q = self.get()->getJointState();
    return pybind11::array(q.dim(), q.p);
  }, "",
    py::arg("joints") = ry::I_StringA() )

  .def("setJointState", [](ry::Config& self, const std::vector<double>& q, const ry::I_StringA& joints){
    arr _q = conv_stdvec2arr(q);
    if(joints.size()){
      self.set()->setJointState(_q, I_conv(joints));
    }else{
      self.set()->setJointState(_q);
    }
  }, "",
    py::arg("q"),
    py::arg("joints") = ry::I_StringA() )

  .def("getFrameNames", [](ry::Config& self){
    return I_conv(self.get()->getFrameNames());
  } )

  .def("getFrameState", [](ry::Config& self){
    arr X = self.get()->getFrameState();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("getFrameState", [](ry::Config& self, const char* frame){
    arr X;
    auto Kget = self.get();
    rai::Frame *f = Kget->getFrameByName(frame, true);
    if(f) X = f->X.getArr7d();
    return pybind11::array(X.dim(), X.p);
  } )

  .def("setFrameState", [](ry::Config& self, const std::vector<double>& X, const ry::I_StringA& frames, bool calc_q_from_X){
    arr _X = conv_stdvec2arr(X);
    _X.reshape(_X.N/7, 7);
    self.set()->setFrameState(_X, I_conv(frames), calc_q_from_X);
  }, "",
    py::arg("X"),
    py::arg("frames") = ry::I_StringA(),
    py::arg("calc_q_from_X") = true )

  .def("feature", [](ry::Config& self, FeatureSymbol fs, const ry::I_StringA& frames) {
    auto Kget = self.get();
    ry::RyFeature F;
//    F.feature = make_shared<::Feature>(symbols2feature(fs, I_conv(frames), Kget));
    F.feature = symbols2feature(fs, I_conv(frames), Kget);
    return F;
  } )

  .def("selectJointsByTag", [](ry::Config& self, const ry::I_StringA& jointGroups){
    auto Kset = self.set();
    Kset->selectJointsByGroup(I_conv(jointGroups), true, true);
    Kset->calc_q();
  } )

  .def("selectJoints", [](ry::Config& self, const ry::I_StringA& joints){
    // TODO: this is joint groups
    // TODO: maybe call joint groups just joints and joints DOFs
    self.set()->selectJointsByName(I_conv(joints));
  } )

  .def("makeObjectsFree", [](ry::Config& self, const ry::I_StringA& objs){
    self.set()->makeObjectsFree(I_conv(objs));
  } )

  .def("view", [](ry::Config& self, const std::string& frame){
    ry::ConfigView view;
    view.view = make_shared<KinViewer>(self, -1, rai::String(frame));
    return view;
  }, "",
    py::arg("frame")="")

  .def("komo_IK", [](ry::Config& self){
      return ry::KOMOpy(self, 0);
  } )

  .def("komo_path",  [](ry::Config& self, double phases, uint stepsPerPhase, double timePerPhase){
      return ry::KOMOpy(self, phases, stepsPerPhase, timePerPhase);
  }, "",
    py::arg("phases"),
    py::arg("stepsPerPhase")=20,
    py::arg("timePerPhase")=5. )

  .def("komo_CGO", [](ry::Config& self, uint numConfigs){
      CHECK_GE(numConfigs, 1, "");
      return ry::KOMOpy(self, numConfigs);
  } )

  .def("lgp", [](ry::Config& self, const std::string& folFileName){
      return ry::LGPpy(self, folFileName);
  } )

  ;

//  py::class_<ry::Display>(m, "Display")
//      .def("update", (void (ry::Display::*)(bool)) &ry::Display::update)
//      .def("update", (void (ry::Display::*)(std::string, bool)) &ry::Display::update);

  //===========================================================================

  py::class_<ry::ConfigView>(m, "ConfigView")
      ;

  //===========================================================================

  py::class_<ry::RyFeature>(m, "Feature")
  .def("eval", [](ry::RyFeature& self, ry::Config& K){
    arr y,J;
    self.feature->phi(y, J, K.get());
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  } )
  .def("eval", [](ry::RyFeature& self, pybind11::tuple& Kpytuple){
    WorldL Ktuple;
    for(uint i=0;i<Kpytuple.size();i++){
      ry::Config& K = Kpytuple[i].cast<ry::Config&>();
      Ktuple.append(&K.set()());
    }

    arr y, J;
    self.feature->order=Ktuple.N-1;
    self.feature->phi(y, J, Ktuple);
    cout <<"THERE!!" <<J.dim() <<endl;
    pybind11::tuple ret(2);
    ret[0] = pybind11::array(y.dim(), y.p);
    ret[1] = pybind11::array(J.dim(), J.p);
    return ret;
  } )
  .def("description", [](ry::RyFeature& self, ry::Config& K){
    std::string s = self.feature->shortTag(K.get()).p;
    return s;
  } )
  ;

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

#define ENUMVAL(pre, x) .value(#x, pre##_##x)

  py::enum_<ObjectiveType>(m, "OT")
      ENUMVAL(OT,none)
      ENUMVAL(OT,f)
      ENUMVAL(OT,sos)
      ENUMVAL(OT,ineq)
      ENUMVAL(OT,eq)
      .export_values();

  py::enum_<rai::ShapeType>(m, "ST")
      ENUMVAL(rai::ST,none)
      ENUMVAL(rai::ST,box)
      ENUMVAL(rai::ST,sphere)
      ENUMVAL(rai::ST,capsule)
      ENUMVAL(rai::ST,mesh)
      ENUMVAL(rai::ST,cylinder)
      ENUMVAL(rai::ST,marker)
      ENUMVAL(rai::ST,pointCloud)
      ENUMVAL(rai::ST,ssCvx)
      ENUMVAL(rai::ST,ssBox)
      .export_values();

  py::enum_<FeatureSymbol>(m, "FS")
      ENUMVAL(FS,none)
      ENUMVAL(FS,position)
      ENUMVAL(FS,positionDiff)
      ENUMVAL(FS,positionRel)
      ENUMVAL(FS,quaternion)
      ENUMVAL(FS,quaternionDiff)
      ENUMVAL(FS,quaternionRel)
      ENUMVAL(FS,pose)
      ENUMVAL(FS,poseDiff)
      ENUMVAL(FS,poseRel)
      ENUMVAL(FS,vectorX)
      ENUMVAL(FS,vectorXDiff)
      ENUMVAL(FS,vectorXRel)
      ENUMVAL(FS,vectorY)
      ENUMVAL(FS,vectorYDiff)
      ENUMVAL(FS,vectorYRel)
      ENUMVAL(FS,vectorZ)
      ENUMVAL(FS,vectorZDiff)
      ENUMVAL(FS,vectorZRel)
      ENUMVAL(FS,scalarProductXX)
      ENUMVAL(FS,scalarProductXY)
      ENUMVAL(FS,scalarProductXZ)
      ENUMVAL(FS,scalarProductYX)
      ENUMVAL(FS,scalarProductYY)
      ENUMVAL(FS,scalarProductYZ)
      ENUMVAL(FS,scalarProductZZ)
      ENUMVAL(FS,gazeAt)

      ENUMVAL(FS,accumulatedCollisions)
      ENUMVAL(FS,jointLimits)
      ENUMVAL(FS,distance)

      ENUMVAL(FS,qItself)

      ENUMVAL(FS,aboveBox)
      ENUMVAL(FS,insideBox)

      ENUMVAL(FS,standingAbove)

      ENUMVAL(FS,physics)
      ENUMVAL(FS,contactConstraints)
      ENUMVAL(FS,energy)

      .export_values();
#undef ENUMVAL

}
