#include "configuration.h"

#include <Kin/TM_PairCollision.h>
#include <Kin/frame.h>

ry::Configuration::~Configuration(){
//  LOG(0) <<"destroy " <<this;
  for(auto& d:displays) d->kin=NULL;
}

void ry::Configuration::addFile(const std::string& file){
  K.set()->addModel(file.c_str());
  for(auto& d:displays) d->gl.update(STRING("addFile '" <<file <<"'"));
}

void ry::Configuration::addFrame(const std::string& name, const std::string& parent, const std::string& args){
  LOG(0) <<"here" <<args <<endl;
  K.writeAccess();
  rai::Frame *p = 0;
  if(parent.size()) p = K()[parent.c_str()];
  rai::Frame *f = new rai::Frame(K());
  f->name = name;
  if(p) f->linkFrom(p);

  Graph _args;
  rai::String(args) >>_args;
  cout <<"ARGUMENTS: " <<_args <<endl;
  f->read(_args);
//  if(args.size()){
//    rai::Shape *s = new rai::Shape(*f);
//    rai::String(args) >>s->type();
//    s->size() = size;
//  }
//  if(pose.size()){
//    rai::String(pose) >>f->Q;
//    if(!p) f->X = f->Q;
//  }
  K.deAccess();
  for(auto& d:displays) d->gl.update(STRING("addFrame '" <<name <<'(' <<parent <<")'"));
}

I_StringA ry::Configuration::getJointNames(){
  return I_conv(K.get()->getJointNames());
}

pybind11::array ry::Configuration::getJointState(const I_StringA& joints){
  arr q;
  if(joints.size()) q = K.get()->getJointState(I_conv(joints));
  else q = K.get()->getJointState();
  return pybind11::array(q.dim(), q.p);
}

void ry::Configuration::setJointState(pybind11::array& q){
  auto buf = q.request();
  CHECK_EQ(buf.ndim, 1, "");

  arr _q((double*)buf.ptr, buf.size);
  K.set()->setJointState(_q);
  rai::String str = "setJointState";
  _q.write(str,"\n");
  for(auto& d:displays) d->gl.update(str);
}

I_StringA ry::Configuration::getFrameNames(){
  return I_conv(K.get()->getFrameNames());
}

pybind11::array ry::Configuration::getFrameState(){
  arr X = K.get()->getFrameState();
  return pybind11::array(X.dim(), X.p);
}

void ry::Configuration::setFrameState(pybind11::array& X){
  auto buf = X.request();
  CHECK_EQ(buf.ndim, 2, "");

  arr _X((double*)buf.ptr, buf.size);
  _X.reshape(buf.size/7, 7);
  K.set()->setFrameState(_X, true);
  for(auto& d:displays) d->gl.update(STRING("setFrameState"));
}

double ry::Configuration::getPairDistance(const char* frameA, const char* frameB){
  K.readAccess();
  TM_PairCollision coll(K(), frameA, frameB, TM_PairCollision::_negScalar, false);
  arr y;
  coll.phi(y, NoArr, K());
  K.deAccess();
  return -y.scalar();
}

ry::Display ry::Configuration::display(){ return Display(this); }

ry::KOMOpy ry::Configuration::komo(){ return KOMOpy(this); }


