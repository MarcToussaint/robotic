#include "komo-py.h"

#include "komo-py.h"

#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>
#include <Kin/TM_qItself.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_qLimits.h>
#include <Kin/proxy.h>
#include <Kin/TM_time.h>
#include <Gui/opengl.h>

#include <LGP/bounds.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);


ry::KOMOpy::KOMOpy(Config& _kin, uint T)
  : kin(_kin), self(make_shared<KOMO>()) {

  self->setModel(kin.get(), true);
  self->world.optimizeTree();
  self->world.calc_q();

  if(T==0){ //IK mode
    self->setIKOpt();
  }else{
    self->setDiscreteOpt(T);
  }
}

ry::KOMOpy::KOMOpy(ry::Config& _kin, double phases, uint stepsPerPhase, double timePerPhase)
  : kin(_kin), self(make_shared<KOMO>()) {

  //  LOG(0) <<"create " <<this;
  self->setModel(kin.get(), true);
  self->world.optimizeTree();
  self->world.calc_q();

  self->setPathOpt(phases, stepsPerPhase, timePerPhase);
}

ry::KOMOpy::~KOMOpy(){
}

void ry::KOMOpy::makeObjectsFree(const I_StringA& objs){
  self->world.makeObjectsFree(I_conv(objs));
}

void ry::KOMOpy::activateCollisionPairs(const std::vector<std::pair<std::string, std::string> >& collision_pairs){
  for (const auto&  pair : collision_pairs) {
    self->activateCollisions(rai::String(pair.first), rai::String(pair.second));
  }
}

void ry::KOMOpy::deactivateCollisionPairs(const std::vector<std::pair<std::string, std::string> >& collision_pairs){
  for (const auto&  pair : collision_pairs) {
    self->deactivateCollisions(rai::String(pair.first), rai::String(pair.second));
  }
}

void ry::KOMOpy::timeOptimization(){
  self->setTimeOptimization();
}

void ry::KOMOpy::clearObjectives(){
  self->clearObjectives();
}

void ry::KOMOpy::addObjective2(const std::vector<int>& confs, const ObjectiveType& type, const FeatureSymbol& feature, const I_StringA& frames, const std::vector<double>& scale, const std::vector<double>& target, int order){
  CHECK(self->denseOptimization, "");
  self->addObjective(convert<double>(intA(confs)), type, feature, I_conv(frames), arr(scale), arr(target), order);
}

void ry::KOMOpy::addObjective(const std::vector<double>& timeInterval, const ObjectiveType& type, const FeatureSymbol& feature, const I_StringA& frames, const std::vector<double>& scale, const std::vector<double>& target, int order){
//  CHECK(!self->denseOptimization, "");
  self->addObjective(arr(timeInterval), type, feature, I_conv(frames), arr(scale), arr(target), order);
}

void ry::KOMOpy::add_grasp(int conf, const char* gripper, const char* object){
  addObjective2({conf}, OT_eq, FS_distance, {gripper, object});
}

void ry::KOMOpy::add_place(int conf, const char* object, const char* table){
  addObjective2({conf}, OT_ineq, FS_aboveBox, {table, object});
  addObjective2({conf}, OT_eq, FS_standingAbove, {table, object});
  addObjective2({conf}, OT_sos, FS_vectorZ, {object}, {}, {0.,0.,1.});
}

void ry::KOMOpy::add_StableRelativePose(const std::vector<int>& confs, const char* gripper, const char* object){
  for(uint i=1;i<confs.size();i++)
    add_restingRelative(confs[0], confs[i], object, gripper);

//  for(uint i=0;i<confs.size();i++) self->configurations(self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}

void ry::KOMOpy::add_StablePose(const std::vector<int>& confs, const char* object){
  for(uint i=1;i<confs.size();i++)
    add_resting(confs[0], confs[i], object);

//  for(uint i=0;i<confs.size();i++) self->configurations(self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}

void ry::KOMOpy::add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper){
  addObjective2({conf1, conf2}, OT_eq, FS_poseDiff, {tableOrGripper, object});
}

void ry::KOMOpy::add_resting(int conf1, int conf2, const char* object){
  addObjective2({conf1, conf2}, OT_eq, FS_pose, {object});
}

void ry::KOMOpy::optimize(){
  self->optimize();
}

int ry::KOMOpy::getT(){
  return self->T;
}

void ry::KOMOpy::getConfiguration(int t){
  kin.writeAccess();
  arr X = self->configurations(t+self->k_order)->getFrameState();
  if(X.d0 > kin().frames.N) X.resizeCopy(kin().frames.N,7);
  kin().setFrameState(X);
//  kin().copyProxies( *self->configurations(t+self->k_order) );
  kin.deAccess();
//  for(auto& d:self->kin.cameras) d->gl.update(STRING("KOMOpy configuration " <<t));
}

//std::string ry::KOMOpy::getReport(){
//  Graph specs = self->getProblemGraph(true);
//  std::stringstream str;
//  str <<specs;
//  return str.str();
//}

Graph ry::KOMOpy::getProblemGraph(){
  return self->getProblemGraph(true);
}

double ry::KOMOpy::getConstraintViolations(){
  Graph R = self->getReport(false);
  return R.get<double>("constraints");
}

double ry::KOMOpy::getCosts(){
  Graph R = self->getReport(false);
  return R.get<double>("sqrCosts");
}

void ry::KOMOpy::display(){
  self->displayPath(true, true);
}
