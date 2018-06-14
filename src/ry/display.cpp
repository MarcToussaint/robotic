#include "display.h"
#include "configuration.h"

ry::KinDisplay::KinDisplay(ry::Kin& _kin) : kin(_kin) { //self(make_shared<KinDisplay_self>(K)){
  gl.add(glStandardScene);
  gl.add(kin.K);
  gl.update();
  kin.displays.append(this);
}

ry::KinDisplay::~KinDisplay(){
  kin.displays.removeValue(this);
}

void ry::KinDisplay::update(bool wait){ if(wait) gl.watch(); else gl.update(); }
