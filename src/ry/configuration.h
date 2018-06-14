#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "display.h"

namespace ry{

  struct Kin{
    Var<rai::KinematicWorld> K;
    rai::Array<KinDisplay*> displays;

    Kin(){}

    void addFile(const std::string& file);

    pybind11::array getJointState();

    KinDisplay* display();
  };

}
