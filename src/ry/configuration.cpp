#include "configuration.h"

ry::Configuration::~Configuration(){
//  LOG(0) <<"destroy " <<this;
  cout <<displays <<endl;
  for(auto& d:displays) d->kin=NULL;
  displays.clear();
}

void ry::Configuration::addFile(const std::string& file){
  K.set()->addModel(file.c_str());
  for(auto& d:displays) d->gl.update(STRING("addFile '" <<file <<"'"));
}

pybind11::array ry::Configuration::getJointState(){
  arr q = K.get()->getJointState();
  return pybind11::array(q.dim(), q.p);
}

ry::Display ry::Configuration::display(){ return Display(this); }


