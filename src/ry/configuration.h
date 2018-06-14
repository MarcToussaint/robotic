#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "types.h"
#include "display.h"
//#include "IK.h"

namespace ry{

  struct Configuration{
    Var<rai::KinematicWorld> K;
    rai::Array<Display_self*> displays;

    Configuration(){}
    ~Configuration();

    //-- editing
    void addFile(const std::string& file);
    void addFrame(const std::string& name, const std::string& parent, const std::string& args);

    //-- set/get state
    I_StringA getJointNames();
    pybind11::array getJointState(const I_StringA& joints);
    void setJointState(pybind11::array& q);

    I_StringA getFrameNames();
    pybind11::array getFrameState();
    void setFrameState(pybind11::array& X);

    //-- queries
    double getPairDistance(const char* frameA, const char* frameB);

    //-- modules
    Display display();
//    IK getIK();
    //camerasim
    //physx
    //swift
    //komo
    //IK
    //LGP
  };

}
