#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "types.h"
#include "display.h"
#include "komo-py.h"

namespace ry{

  struct Configuration{
    Var<rai::KinematicWorld> K;
    rai::Array<Display_self*> displays;

    Configuration(){}
    ~Configuration();

    //-- editing
    void addFile(const std::string& file);
    void addFrame(const std::string& name, const std::string& parent, const std::string& args);
    void editorFile(const std::string& file);

    //-- set/get state
    I_StringA getJointNames();
    pybind11::array getJointState(const I_StringA& joints);
    void setJointState(pybind11::array& q);

    I_StringA getFrameNames();
    pybind11::array getFrameState();
    void setFrameState(pybind11::array& X);

    //-- set/get frame-wise
    FrameInfo getFrameInfo(const char* frame);

    //-- jacobians
    pybind11::array getJacobianPos(const char* frame);
    pybind11::array getJacobianRot(const char* frame);

    //-- collision queries
    double getPairDistance(const char* frameA, const char* frameB);
    I_dict getCollisionReport();

    //-- modules
    Display display();

    //view - simulate rgb & depth images, point clouds, for a cam

    KOMOpy komo();     ///< IK, motion and seq manipulation optimization

    //cgo - more general constraint graph optimization

    //physx - stepping the PhysX simulator

    //bullet - stepping the bullet simulator

    //exec - execute motions on pr2, baxter, kuka; and sync back joint states

    //track - OptiTrack only for now
    
  };

}
