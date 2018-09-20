#include "komo-py.h"
#include "configuration.h"

#include "komo-py.h"

#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>
#include <Kin/TM_qItself.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_qLimits.h>
#include <Kin/proxy.h>
#include <Kin/TM_time.h>

#include <LGP/bounds.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);

ry::KOMOpy_self::KOMOpy_self(ry::Configuration* _kin, uint T)
  : kin(_kin) {
//  LOG(0) <<"create " <<this;
  setModel(kin->K.get(), true);
  world.optimizeTree();
  world.calc_q();

  denseMode = true;
  if(T==0){ //IK mode
    setIKOpt();
  }else{
    setDiscreteOpt(T);
  }
}

ry::KOMOpy_self::KOMOpy_self(ry::Configuration* _kin, double phases, uint stepsPerPhase, double timePerPhase)
  : kin(_kin) {
  //  LOG(0) <<"create " <<this;
  setModel(kin->K.get(), true);
  world.optimizeTree();
  world.calc_q();

  denseMode = false;
//  setPathOpt(phases, stepsPerPhase, timePerPhase);
  setTiming(phases, stepsPerPhase, timePerPhase, 2);
}

ry::KOMOpy_self::~KOMOpy_self(){
//  LOG(0) <<"destroy " <<this;
}

void ry::KOMOpy_self::setDiscreteOpt(uint k){
  maxPhase = double(k);
  stepsPerPhase = 1;
  T = k;
  tau = 1.;
  k_order = 1;
//  setFixEffectiveJoints();
//  setFixSwitchedObjects();
//  setSquaredQVelocities(-1., -1., 1e-2);
  setSquaredQuaternionNorms();
}

/** This is the single fundamental method to define an objective. All other methods do nothing but
 * call this lowest level method.
 *
 * Example:  komo.setObjective({1,2}, OT_sos, new TM_qItself());
 * adds a sum-of-square objective over variables x(1) and x(2), defined by the TM-qItself feature, namely, the difference of q in x(1) and x(2)
 *
 */
Objective *ry::KOMOpy_self::setObjective(const arr& times, ObjectiveType type, Feature *map, const arr &target, double scale){
  Objective *task = addObjective(-1.,-1., map, type);
  if(!denseMode){
    if(!times.N){
      task->setCostSpecs(0, T-1, target, scale);
    }else if(times.N==1){
      task->setCostSpecs(times(0), times(0), stepsPerPhase, T, target, scale);
    }else{
      CHECK_EQ(times.N, 2, "");
      task->setCostSpecs(times(0), times(1), stepsPerPhase, T, target, scale);
    }
  }else{
    intA vars = convert<int,double>(times);
    if(!vars.N){
      task->setCostSpecs(0, T-1, target, scale);
//      CHECK_EQ(T, 1, "you are not in IK mode: you need to specify a variable tuple");
//      vars = {0};
    }else{
      uint order = vars.N-1;
      CHECK_GE(k_order, order, "task requires larger k-order: " <<map->shortTag(world));
      map->order = order;
      task->setCostSpecsDense(vars, target, scale);
    }
  }
  return task;
}

Feature *ry::KOMOpy_self::symbols2feature(FeatureSymbol feat, const StringA &frames, const std::map<std::string, std::vector<double>>& parameters){
#if 1
  return ::symbols2feature(feat, frames, world); //parameters);
#else
  if(!symbols.N) return 0;
  if(symbols(0)=="dist") {  return new TM_PairCollision(world, symbols(1), symbols(2), TM_PairCollision::_negScalar, false); }
  if(symbols(0)=="above") {  return new TM_AboveBox(world, symbols(2), symbols(1), .05); }
  if(symbols(0)=="aboveZ") {
    // Difference in z coordinate values
    double h = .5*(shapeSize(world, symbols(1)) + shapeSize(world, symbols(2)));
    Feature *relPos = new TM_Default(TMT_posDiff, world, symbols(1), rai::Vector(0.,0.,h), symbols(2), NoVector);
    return new TM_LinTrans(relPos, arr(1,3,{0.,0.,1.}), {});
  }

  rai::Vector v1=0,v2=0;
  if(parameters.find("v1")!=parameters.end()) v1.set( parameters.at("v1") );
  if(parameters.find("v2")!=parameters.end()) v2.set( parameters.at("v2") );

  if(symbols(0)=="pos") {  return new TM_Default(TMT_pos, world, symbols(1), v1); }
  if(symbols(0)=="posRel") {  return new TM_Default(TMT_pos, world, symbols(1), v1, symbols(2), v2); }
  if(symbols(0)=="posDiff") {  return new TM_Default(TMT_posDiff, world, symbols(1), v1, symbols(2), v2); }

  if(symbols(0)=="vec") {  return new TM_Default(TMT_vec, world, symbols(1), v1); }
  if(symbols(0)=="vecRel") {  return new TM_Default(TMT_vec, world, symbols(1), v1, symbols(2), v2); }
  if(symbols(0)=="vecDiff") {  return new TM_Default(TMT_vecDiff, world, symbols(1), v1, symbols(2), v2); }

  if(symbols(0)=="quat") {  return new TM_Default(TMT_quat, world, symbols(1)); }
  if(symbols(0)=="quatRel") {  return new TM_Default(TMT_quat, world, symbols(1), NoVector, symbols(2), NoVector); }
  if(symbols(0)=="quatDiff") {  return new TM_Default(TMT_quatDiff, world, symbols(1), NoVector, symbols(2), NoVector); }

  if(symbols(0)=="pose") {  return new TM_Default(TMT_pose, world, symbols(1)); }
  if(symbols(0)=="poseDiff") {  return new TM_Default(TMT_poseDiff, world, symbols(1), NoVector, symbols(2), NoVector); }

  if(symbols(0)=="prod") {  return new TM_Default(TMT_vecAlign, world, symbols(1), v1, symbols(2), v2); }

  if(symbols(0)=="gazeAt") { return new TM_Default(TMT_gazeAt, world, symbols(1), v1, symbols(2), v2); }

  double margin=.0;
  if(parameters.find("margin")!=parameters.end()) margin = parameters.at("margin")[0];

  if(symbols(0)=="coll") {  return new TM_Proxy(TMT_allP, {}, margin); }
  if(symbols(0)=="limits") {  return new LimitsConstraint(.05); }

  if(symbols(0)=="qRobot") {
    if(symbols.N==1) return new TM_qItself();
    return new TM_qItself(QIP_byJointNames, symbols({1,-1}), world);
  }

  HALT("can't interpret feature symbols: " <<symbols);
  return 0;
#endif
}

arr ry::KOMOpy_self::getPose(uint t, const rai::String& name){
  return configurations(t+k_order)->getFrameByName(name)->X.getArr7d();
}

arr ry::KOMOpy_self::getRelPose(uint t, const rai::String& from, const rai::String& to){
  rai::Transformation& A = configurations(t+k_order)->getFrameByName(from)->X;
  rai::Transformation& B = configurations(t+k_order)->getFrameByName(to)->X;
  return (B/A).getArr7d();
}


/** Same as setObjective, but uses symbols to refer to a predefined feature, and a generic parameter list
 *
 * Example: komo.setObjective({0,1}, OT_eq, { "pose", "object", "hand"}
 * to enforce that the relative object-hand pose is equal in x(0) and x(1)
 *
 * komo.setObjective({2}, OT_sos, { "vec", "object" }, { {"target", {0.,0.,1.} } );
 *
 */
void ry::KOMOpy_self::setObjective(const arr& times, ObjectiveType type, FeatureSymbol feat, const StringA &frames, const std::map<std::string, std::vector<double>> &parameters){
  arr target;
  double scale=1e1;
  if(parameters.find("scale")!=parameters.end()) scale = parameters.at("scale")[0];
  if(parameters.find("target")!=parameters.end()) target = parameters.at("target");

//  rai::Enum<ObjectiveType> type;
//  featureSymbols(0) >>type;
//  StringA symbols = featureSymbols({1,-1});

  Objective *t = setObjective(times, type, symbols2feature(feat, frames, parameters), target, scale);

  if(parameters.find("order")!=parameters.end()) t->map->order = (uint)parameters.at("order")[0];
}


ry::KOMOpy::KOMOpy(ry::Configuration* _kin, uint T)
  : self(make_shared<ry::KOMOpy_self>(_kin, T)) {
}

ry::KOMOpy::KOMOpy(ry::Configuration* _kin, double phases, uint stepsPerPhase, double timePerPhase)
  : self(make_shared<ry::KOMOpy_self>(_kin, phases, stepsPerPhase, timePerPhase)) {
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
  auto *jt = new rai::Joint(*self->world["world"]);
  jt->type = rai::JT_time;
  jt->H = 0.;
  self->addObjective(0., -1., new TM_Time(), OT_sos, {}, 1e1, 1); //smooth time evolution
  self->addObjective(0., -1., new TM_Time(), OT_sos, {self->tau}, 1e-1, 0); //prior on timing
}

void ry::KOMOpy::clearObjectives(){
  self->clearObjectives();
}

void ry::KOMOpy::addObjective(const std::vector<int>& confs, const std::vector<double>& timeInterval, const std::string& type, const std::string& feature, const I_StringA& frames, const std::vector<double>& scale, const std::vector<double>& target, I_args parameters){
  rai::Enum<ObjectiveType> __type;
  __type = type.c_str();

  rai::Enum<FeatureSymbol> feat;
  feat = feature.c_str();

  if(scale.size()) parameters["scale"] = scale;
  if(target.size()) parameters["target"] = target;
  if(timeInterval.size()){
    CHECK_EQ(confs.size(), 0, "");
    self->setObjective(arr(timeInterval), __type, feat, I_conv(frames), parameters);
  }else{
    CHECK_EQ(timeInterval.size(), 0, "");
    self->setObjective(convert<double>(intA(confs)), __type, feat, I_conv(frames), parameters);
  }
}

//void ry::KOMOpy::addObjectives2(const Graph& features){
//  for(Node* n : features) {
//    Graph& feature = n->graph();

//    arr times;
//    if(feature["time"]) times = feature.get<arr>("time");

//    StringA symbols = feature.get<StringA>("feature");

//    std::map<std::string, std::vector<double>> parameters;
//    for (Node *p:feature){
//      if(p->isOfType<arr>()){
//        parameters.insert(std::make_pair(p->keys.last().p, p->get<arr>()));
//      }
//      if(p->isOfType<double>()){
//        parameters.insert(std::make_pair(p->keys.last().p, ARR(p->get<double>())));
//      }
//    }

//    self->setObjective(times, symbols, parameters);
//  }
//}

void ry::KOMOpy::addObjectives(const I_features& features){
  for (const I_feature& feature : features) {
    arr times;
    times = std::get<0>(feature);

    StringA symbols = I_conv(std::get<1>(feature));
    rai::Enum<ObjectiveType> type;
    type = symbols.popFirst();

    rai::Enum<FeatureSymbol> feat;
    feat = symbols.popFirst();

    std::map<std::string, std::vector<double>> parameters;
    for (const auto& x : std::get<2>(feature)) {
      parameters.insert(std::make_pair(x.first, conv_stdvec2arr(x.second)));
    }

    self->setObjective(times, type, feat, symbols, parameters);
  }
}

void ry::KOMOpy::add_grasp(int conf, const char* gripper, const char* object){
  addObjective({conf}, {}, "eq", "distance", {gripper, object});
}

void ry::KOMOpy::add_place(int conf, const char* object, const char* table){
  addObjective({conf}, {}, "ineq", "aboveBox", {table, object});
  addObjective({conf}, {}, "eq", "standingAbove", {table, object});
  addObjective({conf}, {}, "sos", "vectorZ", {object}, {}, {0.,0.,1.});
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
  addObjective({conf1, conf2}, {}, "eq", "poseDiff", {tableOrGripper, object});
}

void ry::KOMOpy::addSkeleton(const std::vector<double>& times, ry::I_StringA symbols){
  CHECK_EQ(times.size(), 2, "");
  self->S.append(SkeletonEntry(I_conv(symbols), times[0], times[1]));
}

void ry::KOMOpy::setSkeleton(){
  self->setSkeleton(self->S);
}

void ry::KOMOpy::skeleton2bound(){
  ::skeleton2Bound(*self, BD_path, self->S, self->world, self->world);
}

void ry::KOMOpy::add_resting(int conf1, int conf2, const char* object){
  addObjective({conf1, conf2}, {}, "eq", "pose", {object});
}

void ry::KOMOpy::optimize(){
  self->optimize();
}

int ry::KOMOpy::getT(){
  return self->T;
}

void ry::KOMOpy::getConfiguration(int t){
  self->kin->K.writeAccess();
  arr X = self->configurations(t+self->k_order)->getFrameState();
  if(X.d0 > self->kin->K().frames.N) X.resizeCopy(self->kin->K().frames.N,7);
  self->kin->K().setFrameState(X);
  self->kin->K().copyProxies( *self->configurations(t+self->k_order) );
  self->kin->K.deAccess();
  for(auto& d:self->kin->cameras) d->gl.update(STRING("KOMOpy configuration " <<t));
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
  self->gl->clear();
  NIY; //self->gl->add(*self);
  self->gl->update();
}
