#include "display.h"
#include "configuration.h"

ry::Display::Display(ry::Configuration* _kin)
  : self(make_shared<Display_self>(_kin)) { //self(make_shared<KinDisplay_self>(K)){
}

ry::Display::~Display(){
}

void ry::Display::update(bool wait){
  if(wait) self->gl.watch();
  else self->gl.update();
}

void ry::Display::update(std::string txt, bool wait){
  if(wait) self->gl.watch(txt.c_str());
  else self->gl.update(txt.c_str());
}

ry::Display_self::Display_self(ry::Configuration* _kin) : kin(_kin) {
//  LOG(0) <<"create " <<this;
  kin->displays.append(this);

  gl.add(glStandardScene);
  gl.add(kin->K);
  gl.camera.setDefault();
  gl.update();
}

ry::Display_self::~Display_self(){
//  LOG(0) <<"destroy " <<this;
  if(kin) kin->displays.removeValue(this);
}
