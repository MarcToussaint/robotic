#include "configuration.h"

void ry::Kin::addFile(const std::string& file){
  K.set()->addModel(file.c_str());
  for(auto *d:displays) d->update(false);
}

pybind11::array ry::Kin::getJointState(){
  arr q = K.get()->getJointState();
  return pybind11::array(q.dim(), q.p);
}

ry::KinDisplay*ry::Kin::display(){ return new KinDisplay(*this); }


