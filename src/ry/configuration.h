#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "types.h"
#include "camera.h"
#include "komo-py.h"
#include "lgp-py.h"

/* TODO:
 *
 * delFrame
 * arguments should always be I_arr, not ny.array
 * IK.optimize()  should not push back -> getResult
 */

namespace ry{

  struct Configuration{
    Var<rai::KinematicWorld> K;
    rai::Array<Camera_self*> cameras;
    arr stack;

    Configuration();
    ~Configuration();

    //--
    Configuration copy();

    //-- editing
    void clear();
    void addFile(const std::string& file);
    void addFrame(const std::string& name, const std::string& parent, const std::string& args);
    void delFrame(const std::string& name);
    void editorFile(const std::string& file);

    //-- set/get state
    I_StringA getJointNames();
    pybind11::array getJointState(const I_StringA& joints);
    void setJointState(pybind11::array& q, const I_StringA& joints);
    void setJointState(const I_arr& q, const I_StringA& joints);

    I_StringA getFrameNames();
    pybind11::array getFrameState();
    pybind11::array getFrameState(const char* frame);
    void setFrameState(pybind11::array& X, const I_StringA& frames, bool calc_q_from_X);

    void stash();
    void pop();

    //-- modulate DOFs
    void useJointGroups(const I_StringA& jointGroups);
    void setActiveJoints(const I_StringA& joints);
    void makeObjectsFree(const I_StringA& objs);

    //-- set/get frame-wise
    FrameInfo getFrameInfo(const char* frame);

    //-- jacobians
    pybind11::array getJacobianPos(const char* frame);
    pybind11::array getJacobianRot(const char* frame);

    //-- collision queries
    double getPairDistance(const char* frameA, const char* frameB);
//    I_dict getCollisionReport();

    //-- modules
    /// simulate rgb & depth images, point clouds, for a cam
    Camera camera(const std::string& frame={}, bool _renderInBackground=false);

    /// IK, motion and seq manipulation optimization
    KOMOpy komo_IK();    ///< to optimize a single configuration
    KOMOpy komo_path(double phases, uint stepsPerPhase=20, double timePerPhase=5.);  ///< to optimize a k-order Markov path
    KOMOpy komo_CGO(uint numConfigurations);   ///< to optimize a (non-sequential) constraint graph

    LGPpy lgp(const std::string& folFileName="fol.g");

    //physx - stepping the PhysX simulator
    //bullet - stepping the bullet simulator

    //exec - execute motions on pr2, baxter, kuka; and sync back joint states

    //track - OptiTrack only for now
    
  };

}
