#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "display.h"

namespace ry{

  struct Configuration{
    Var<rai::KinematicWorld> K;
    rai::Array<Display_self*> displays;

    Configuration(){}
    ~Configuration();

    void addFile(const std::string& file);

    pybind11::array getJointState();

    Display display();
  };

}
