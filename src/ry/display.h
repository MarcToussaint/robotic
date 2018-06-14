#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

namespace ry{

  template<class T> using ptr=std::shared_ptr<T>;
  struct Kin;

  struct KinDisplay{
    Kin& kin;
    OpenGL gl;

    KinDisplay(struct Kin& _kin);
    ~KinDisplay();

    void update(bool wait=false);
  };

}
