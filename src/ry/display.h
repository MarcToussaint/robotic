#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "types.h"

namespace ry{

  struct Configuration;

  struct Display_self{
    Display_self(Configuration* _kin);
    ~Display_self();
    Configuration* kin=0;
    OpenGL gl;
  };

  struct Display{
    ptr<Display_self> self;

    Display(Configuration* _kin);
    ~Display();

    void update(bool wait=false);
    void update(std::string txt, bool wait=false);
  };

}
