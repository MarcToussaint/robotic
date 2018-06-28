#pragma once

#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <pybind11/numpy.h>

#include "types.h"

//#include "configuration.h"

//======================================================================================================================

namespace ry{

  struct Configuration;

  struct Camera_self : GLDrawer{
    Camera_self(Configuration* _kin);
    ~Camera_self();
    Configuration *kin;
    rai::Frame *frame;
    OpenGL gl;
    rai::Camera cam;
    uint width=640, height=480;
    byteA backgroundImage;
    byteA captureImage;
    floatA captureDepth;
    bool renderInBackground;
    void glDraw(OpenGL &);
  };

  struct Camera {
    ptr<Camera_self> self;

    Camera(Configuration* _kin, const rai::String& frame={}, bool _renderInBackground=false);
    ~Camera(){}

    //-- set camera parameters (the pose is given by the frame)
    void set(uint width, uint height, double focalLength=1., double zNear=.1, double zFar=10.);
    void setOrthographic(uint width, uint height, double zNear, double zFar);
    void setKinect();
    void setBasler();
    void setBackgroundImage(const byteA& img);

    void update(bool wait=false);
    void update(std::string txt, bool wait=false);

    //-- modulate what is drawn
    void drawFrames(StringA& frames);
    void doNotDrawFrames(StringA& frames);

    //-- get the images, depth, point cloud, and object segmentation
    void getImage(byteA& img);
    void getImageAndTrueDepth(byteA& rgb, arr& depth);
    void getImageAndScaleDepth(byteA& rgb, arr& depth);
    void getImageAndKinectDepth(byteA& rgb, arr& depth);
    void getPointCloud(arr& points, byteA& colors);
    void getFrameSegmentation(byteA& segmentation);
  };

}

//======================================================================================================================
