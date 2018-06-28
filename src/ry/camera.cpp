#include "camera.h"
#include "configuration.h"

#include <GL/gl.h>


ry::Camera_self::Camera_self(ry::Configuration* _kin) : kin(_kin) {
//  LOG(0) <<"create " <<this;
  kin->cameras.append(this);
}

ry::Camera_self::~Camera_self(){
//  LOG(0) <<"destroy " <<this;
  if(kin) kin->cameras.removeValue(this);
}

ry::Camera::Camera(Configuration* _kin, const rai::String& frame, bool _renderInBackground)
  : self(make_shared<Camera_self>(_kin)){

  if(frame.N){
    self->frame = self->kin->K.get()->getFrameByName(frame, true);
  }
  self->renderInBackground = _renderInBackground;

  double d;
  arr z;
  if(self->frame){
    if(self->frame->ats.get<double>(d,"focalLength")) self->cam.setFocalLength(d);
    if(self->frame->ats.get<double>(d,"width")) self->width = (uint)d;
    if(self->frame->ats.get<double>(d,"height")) self->height = (uint)d;
    if(self->frame->ats.get<arr>(z,"zrange")) self->cam.setZRange(z(0), z(1));
  }else{
    self->cam.setDefault();
    self->gl.camera = self->cam;
  }

  self->gl.add(*self);

  self->gl.text = STRING("Camera '" <<(frame.N?frame:"<user>") <<"'");
  if(!self->renderInBackground){
    update();
  }
}

void ry::Camera::set(uint width, uint height, double focalLength, double zNear, double zFar){
  self->width = width;
  self->height = height;
  self->cam.setFocalLength(focalLength);
  self->cam.setZRange(zNear, zFar);
  self->gl.text = STRING("Camera set");
  update();
}

void ry::Camera::update(std::string txt, bool wait){
  self->gl.text = txt.c_str();
  update(wait);
}

void ry::Camera::update(bool wait){
  if(self->backgroundImage.N) self->gl.background = self->backgroundImage;
  if(self->frame){
    self->cam.X = self->frame->X;
    self->gl.camera = self->cam;
  }

  if(!self->renderInBackground){
    self->gl.update(NULL, true, true, true);
  }else{
    self->gl.renderInBack(true, true, self->width, self->height);
  }

  if(wait) self->gl.watch();
}

void ry::Camera_self::glDraw(OpenGL& gl){
  glStandardScene(NULL);
  kin->K.set()->glDraw(gl);

  captureImage.resize(height, width, 3);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);
  captureDepth.resize(height, width);
  glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, captureDepth.p);
}

#if 0


void Simulator::addDefaultCameraWithBackgroundImage(const char *imageFile)
{
  Sensor& sen = sensors.append();
  sen.name = "default";
  rai::Camera& cam = sen.cam;
  sen.width=640;
  sen.height=480;
  cam.setKinect();
  cam.setWHRatio(sen.width/sen.height);
  cam.X.setZero();
  cam.X.pos={-.65,.65,2.40};
  cam.X.addRelativeRotationDeg(3,0.,0.,1.);
  byteA img;
  read_png(img, imageFile, false);
  sen.backgroundImage = img;
}

void Simulator::addBaslerCamera(){
  Sensor& sen = sensors.append();
  sen.name = "basler";
  rai::Camera& cam = sen.cam;
  sen.width=2046;
  sen.height=2046;
  cam.setKinect();

  cam.setZero();
  cam.setPosition(0., 0., 0.);
  cam.focus(0., 0., 5.);
  cam.setZRange(.1, 50.);
  cam.setFocalLength(580./480.);

  cam.setWHRatio(sen.width/sen.height);
  cam.X.setZero();
  cam.X.pos={-.65,.65,2.40};
  cam.X.addRelativeRotationDeg(3,0.,0.,1.);
}


void Simulator::addToteOrthoCamera()
{
  Sensor& sen = sensors.append();
  rai::Camera& cam = sen.cam;
  sen.name = "tote_ortho";
  //set and orthographic 100x100 camera, 10cm wide
  sen.width=400;
  sen.height=300;
  cam.setHeightAbs(.6); //width in meters
  cam.zNear=.5; //range of one meter
  cam.zFar =1.5;
  cam.X = K["tote_base_link"]->X; //pose of the tote
  cam.X.pos.z += 1.; // 1 meter above tote
}


void Simulator::computeImageAndDepth()
{
  gl.clear();
  gl.add(glStandardLight);
  gl.add(K);
  // Render images for RGB and depth
  if (show_GUI)
    gl.update(NULL, true, true, true);
  else
    gl.renderInBack(true, true, gl.width, gl.height);
  image = gl.captureImage;
  depth = gl.captureDepth;
  flip_image(image);
  flip_image(depth);
}

void Simulator::computeSegmentation()
{
  gl.clear();
  gl.add(K);
  gl.setClearColors(1, 1, 1, 0);
  gl.drawMode_idColor = true;
  Geo_mesh_drawColors = false;
  if (show_GUI)
    gl.update(NULL, true, true, true);
  else
    gl.renderInBack(true, true, gl.width, gl.height);
  segmentation = gl.captureImage;
  flip_image(segmentation);
  gl.drawMode_idColor = false;
  Geo_mesh_drawColors = true;
}

void glDisableLight(void*) {
  glDisable(GL_LIGHTING);
}

void Simulator::computeKinectPointCloud()
{
  // compute uint16 depth (raw Kinect signal)
  kinect_depth.resize(depth.d0, depth.d1);
  rnd.clockSeed();
#if 0 //mt: no, don't add noise here already! (I think.) if we can generate noise-free data, let's do that first!
  // conv. from [m] -> [mm] + add kinect noise
  for(uint i = 0; i < depth.N; i++)
    kinect_depth.elem(i) = (uint16_t) std::max(gl.camera.glConvertToTrueDepth(depth.elem(i)) * 1000. + 10 *
                                               rnd.gauss(), 0.); // Make sure we are not below 0 because of the gauss noise;
#else
  for(uint i = 0; i < depth.N; i++) kinect_depth.elem(i) = (uint16_t) gl.camera.glConvertToTrueDepth(depth.elem(i)) * 1000.;
#endif

  // compute point cloud
  depthData2pointCloud(kinect_pcl, kinect_depth);
}

void Simulator::saveMainCameraImages()
{
  rai::system(STRING("mkdir -p " << output_dir));

  // Define our camera
  selectSensor("default");

  // save image and depth
  computeImageAndDepth();
  write_png(image, STRING(output_dir << "kinect_final_rgb.png"), false);
  write_png(convert<byte>(255.f * depth), STRING(output_dir << "kinect_final_d.png"), false);

  //save kinect signals and point cloud
  computeKinectPointCloud();
  FILE(STRING(output_dir << "kinect_depth.raw")) << kinect_depth;
  FILE(STRING(output_dir << "kinect_pcl.raw")) << kinect_pcl;

  // Save segmentation image
  computeSegmentation();
  write_png(segmentation, STRING(output_dir << "segmentation.png"), false);

  // save info on which segments come from which meshes
  ofstream index(STRING(output_dir << "segmentIndex.txt"));
  byte indexRgb[3];
  for(rai::Frame *f : asin_frames)
  {
    id2color(indexRgb, f->ID);
    index << f->ID << ' ' << (int)indexRgb[0] << ' ' << (int)indexRgb[1] << ' ' << (int)indexRgb[2] << ' ' <<
             f->name << endl;
  }
  index.close();
}

#endif
