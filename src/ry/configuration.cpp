#include "configuration.h"

#include <Kin/TM_PairCollision.h>
#include <Kin/frame.h>
#include <Kin/proxy.h>

ry::Configuration::Configuration(){
//  LOG(0) <<"create " <<this;
}

ry::Configuration::~Configuration(){
//  LOG(0) <<"destroy " <<this;
  for(auto& d:cameras) d->kin=NULL;
}

void ry::Configuration::addFile(const std::string& file){
  K.set()->addModel(file.c_str());
  for(auto& d:cameras) d->gl.update(STRING("addFile '" <<file <<"'"));
}

void ry::Configuration::addFrame(const std::string& name, const std::string& parent, const std::string& args){
//  LOG(0) <<"here" <<args;
  K.writeAccess();
  rai::Frame *p = 0;
  if(parent.size()) p = K()[parent.c_str()];
  rai::Frame *f = new rai::Frame(K());
  f->name = name;
  if(p) f->linkFrom(p);

  rai::String(args) >>f->ats;
//  cout <<"ARGUMENTS: " <<_args <<endl;
  f->read(f->ats);
//  if(f->shape) f->shape->geom->createMeshes();
//  if(args.size()){
//    rai::Shape *s = new rai::Shape(*f);
//    rai::String(args) >>s->type();
//    s->size() = size;
//  }
//  if(pose.size()){
//    rai::String(pose) >>f->Q;
//    if(!p) f->X = f->Q;
//  }

  if(f->parent) f->X = f->parent->X * f->Q;
  K.deAccess();
  for(auto& d:cameras) d->gl.update(STRING("addFrame '" <<name <<'(' <<parent <<")'"));
}

void ry::Configuration::editorFile(const std::string& filename){
  cout <<"Edit the configuration in the editor. Whenever you save, the display will update. Wrong syntax errors will be displayed here. Hit SPACE in the window to animate, ENTER to force reload after error, q to exit this mode." <<endl;
  K.writeAccess();
  K().clear();
  K().addModel(filename.c_str());
  rai::system(STRING("emacs " <<filename <<" &"));
  {
    OpenGL gl;
    gl.add(glStandardScene);
    gl.add(K());
    editConfiguration(filename.c_str(), K(), gl);
  }
  K.deAccess();
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
  for(auto& d:cameras) d->gl.update(str);
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
  K.set()->setFrameState(_X);
  for(auto& d:cameras) d->gl.update(STRING("setFrameState"));
}

void ry::Configuration::stash(){
  arr X = K.get()->getFrameState();
  stack.append(X);
  stack.reshape(stack.N/X.N, X.d0, 7);
}

void ry::Configuration::pop(){
  arr X = stack[-1];
  stack.resizeCopy(stack.d0-1, stack.d1, stack.d2);
  K.set()->setFrameState(X);
  for(auto& d:cameras) d->gl.update(STRING("pop"));
}

double ry::Configuration::getPairDistance(const char* frameA, const char* frameB){
  K.readAccess();
  TM_PairCollision coll(K(), frameA, frameB, TM_PairCollision::_negScalar, false);
  arr y;
  coll.phi(y, NoArr, K());

  rai::Proxy& proxy = K().proxies.append();
  proxy.a = K().frames(coll.i);
  proxy.b = K().frames(coll.j);

  proxy.d = coll.coll->distance;
  proxy.normal = coll.coll->normal;
  arr P1 = coll.coll->p1;
  arr P2 = coll.coll->p2;
  if(coll.coll->rad1>0.) P1 -= coll.coll->rad1*coll.coll->normal;
  if(coll.coll->rad2>0.) P2 += coll.coll->rad2*coll.coll->normal;
  proxy.posA = P1;
  proxy.posB = P2;

  K.deAccess();
  for(auto& d:cameras) d->gl.update(STRING("getPairDistance " <<frameA <<' ' <<frameB <<" = " <<-y.scalar()));
  return -y.scalar();
}

//ry::Camera ry::Configuration::display(){ return Camera(this); }

ry::Camera ry::Configuration::camera(const std::string& frame, bool _renderInBackground){
  return Camera(this, rai::String(frame), _renderInBackground);
}

ry::KOMOpy ry::Configuration::komo_IK(){
  return KOMOpy(this, 0);
}

ry::KOMOpy ry::Configuration::komo_path(double phases, uint stepsPerPhase, double timePerPhase){
  return KOMOpy(this, phases, stepsPerPhase, timePerPhase);
}

ry::KOMOpy ry::Configuration::komo_CGO(uint numConfigurations){
  CHECK_GE(numConfigurations, 1, "");
  return KOMOpy(this, numConfigurations);
}

ry::LGPpy ry::Configuration::lgp(const std::string& folFileName){
  return LGPpy(this, folFileName);
}


