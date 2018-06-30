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

//  for (const auto&  pair : collision_pairs) {
//    komo.activateCollisions(rai::String(pair.first), rai::String(pair.second));
//  }
//  if (checkCollisions) {
//    komo.setCollisions(false); // Soft constraint
//  }
//  if (checkLimits) {
//    komo.setLimits(true, .05, 1e-2);
  //  }
}

ry::KOMOpy_self::KOMOpy_self(ry::Configuration* _kin, double phases, uint stepsPerPhase, double timePerPhase)
  : kin(_kin) {
  setModel(kin->K.get(), false);
  world.optimizeTree();
  world.calc_q();

  denseMode = false;
  setPathOpt(phases, stepsPerPhase, timePerPhase);
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
Task *ry::KOMOpy_self::setObjective(const arr& times, ObjectiveType type, TaskMap *map, const arr &target, double scale){
  Task *task = addTask(map->shortTag(world), map, type);
  if(!denseMode){
    if(!times.N){
      task->setCostSpecs(-1, -1, target, scale);
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

TaskMap *ry::KOMOpy_self::symbols2feature(const StringA &symbols, const std::map<std::string, std::vector<double>>& parameters){
  if(!symbols.N) return 0;
  if(symbols(0)=="dist") {  return new TM_PairCollision(world, symbols(1), symbols(2), TM_PairCollision::_negScalar, false); }
  if(symbols(0)=="above") {  return new TM_AboveBox(world, symbols(2), symbols(1), .05); }
  if(symbols(0)=="aboveZ") {
    double h = .5*(shapeSize(world, symbols(1)) + shapeSize(world, symbols(2)));
    TaskMap *relPos = new TM_Default(TMT_posDiff, world, symbols(1), rai::Vector(0.,0.,h), symbols(2), NoVector);
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

  if(symbols(0)=="qRobot") { return new TM_qItself(); }

  HALT("can't interpret feature symbols: " <<symbols);
  return 0;
}

Graph ry::KOMOpy_self::getProblemGraph(bool includeValues){
  Graph K;
  //header
  Graph& g = K.newSubgraph({"KOMO_specs"}) -> graph();
  g.newNode<uint>({"x_dim"}, {}, x.N);
  g.newNode<uint>({"T"}, {}, T);
  g.newNode<uint>({"k_order"}, {}, k_order);
  g.newNode<double>({"tau"}, {}, tau);
//  uintA dims(configurations.N);
//  for(uint i=0; i<configurations.N; i++) dims(i)=configurations(i)->q.N;
//  g.newNode<uintA>({"q_dims"}, {}, dims);
//  arr times(configurations.N);
//  for(uint i=0; i<configurations.N; i++) times(i)=configurations(i)->frames.first()->time;
//  g.newNode<double>({"times"}, {}, times);
  g.newNode<bool>({"useSwift"}, {}, useSwift);

  //nodes for each configuration
  for(uint s=0;s<configurations.N;s++){
    Graph& g = K.newSubgraph({STRING((int)s-(int)k_order)}) -> graph();
    g.newNode<uint>({"dim"}, {}, configurations(s)->q.N);
    g.newNode<double>({"tau"}, {}, configurations(s)->frames.first()->time);
  }

  NodeL configs = K.list();
  configs.remove(0);

  //objectives
  uint t_count=0;
  for(Task* task:tasks){
    CHECK(task->prec.nd==1,"");
    for(uint t=0;t<task->prec.N;t++){
      if(task->prec(t)){
//        Graph& g = K.newSubgraph({}, configs.sub(i+k_order-t->map->order,i+k_order)) -> graph();
        Graph& g = K.newSubgraph({}, configs.sub(convert<uint>(task->vars[t]+(int)k_order))) -> graph();
        g.newNode<rai::String>({"type"}, {}, STRING(task->type));
        g.newNode<double>({"scale"}, {}, task->prec(t));
        if(task->target.N) g.newNode<arr>({"target"}, {}, task->target);
        g.copy(task->map->getSpec(world), true);
        if(includeValues){
          arr y;
          task->map->phi(y, NoArr, configurations({t,t+k_order}));
          g.newNode<arr>({"value"}, {}, y);
        }
      }
    }
    t_count++;
  }

  if(switches.N) HALT("not implemented for switches yet");
  if(flags.N) HALT("not implemented for flags yet");

  return K;
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
void ry::KOMOpy_self::setObjective(const arr& times, const StringA &featureSymbols, const std::map<std::string, std::vector<double>> &parameters){
  arr target;
  double scale=1e1;
  if(parameters.find("scale")!=parameters.end()) scale = parameters.at("scale")[0];
  if(parameters.find("target")!=parameters.end()) target = parameters.at("target");

  rai::Enum<ObjectiveType> type;
  featureSymbols(0) >>type;
  StringA symbols = featureSymbols({1,-1});

  Task *t = setObjective(times, type, symbols2feature(symbols, parameters), target, scale);

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

void ry::KOMOpy::setCollionPairs(const std::vector<std::pair<std::string, std::string> >& collision_pairs){
  for (const auto&  pair : collision_pairs) {
    self->activateCollisions(rai::String(pair.first), rai::String(pair.second));
  }
}

void ry::KOMOpy::clearObjectives(){
  self->clearTasks();
}

void ry::KOMOpy::addObjective(const std::vector<int>& confs, const std::vector<double>& timeInterval, const std::string& type, const std::string& feature, const I_StringA& frames, const std::vector<double>& scale, const std::vector<double>& target, I_args parameters){
  StringA Feat;
  Feat.append(rai::String(type));
  Feat.append(rai::String(feature));
  Feat.append(I_conv(frames));
  if(scale.size()) parameters["scale"] = scale;
  if(target.size()) parameters["target"] = target;
  if(timeInterval.size()){
    CHECK_EQ(confs.size(), 0, "");
    self->setObjective(arr(timeInterval), Feat, parameters);
  }else{
    CHECK_EQ(timeInterval.size(), 0, "");
    self->setObjective(convert<double>(intA(confs)), Feat, parameters);
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

    std::map<std::string, std::vector<double>> parameters;
    for (const auto& x : std::get<2>(feature)) {
      parameters.insert(std::make_pair(x.first, conv_stdvec2arr(x.second)));
    }

    self->setObjective(times, symbols, parameters);
  }
}

void ry::KOMOpy::add_grasp(int conf, const char* gripper, const char* object){
  addObjective({conf}, {}, "eq", "dist", {gripper, object});
}

void ry::KOMOpy::add_place(int conf, const char* object, const char* table){
  addObjective({conf}, {}, "ineq", "above", {table, object});
  addObjective({conf}, {}, "eq", "aboveZ", {table, object});
  addObjective({conf}, {}, "sos", "vec", {object}, {}, {0.,0.,1.}, {{"v1",{0.,0.,1.}}});
}

void ry::KOMOpy::add_GraspDecisionVariable(const std::vector<int>& confs, const char* gripper, const char* object){
  for(uint i=1;i<confs.size();i++)
    add_restingRelative(confs[0], confs[i], object, gripper);

//  for(uint i=0;i<confs.size();i++) self->configurations(self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}

void ry::KOMOpy::add_PoseDecisionVariable(const std::vector<int>& confs, const char* object){
  for(uint i=1;i<confs.size();i++)
    add_resting(confs[0], confs[i], object);

//  for(uint i=0;i<confs.size();i++) self->configurations(self->k_order+confs[i]) -> makeObjectsFree({object});
  self->world.makeObjectsFree({object});
}

void ry::KOMOpy::add_restingRelative(int conf1, int conf2, const char* object, const char* tableOrGripper){
  addObjective({conf1, conf2}, {}, "eq", "poseDiff", {tableOrGripper, object});
}

void ry::KOMOpy::add_resting(int conf1, int conf2, const char* object){
  addObjective({conf1, conf2}, {}, "eq", "pose", {object});
}

void ry::KOMOpy::optimize(){
  self->reset();
  self->reportProblem();

  self->run(self->denseMode);

  Graph specs = self->getProblemGraph();
  cout <<specs <<endl;
  cout <<self->getReport(false) <<endl; // Enables plot
//  while(self->displayTrajectory());


  self->kin->K.set()->setFrameState(self->configurations(-1)->getFrameState());
  for(auto& d:self->kin->cameras) d->gl.update(STRING("KOMOpy::optimization end pose"));
}

void ry::KOMOpy::getConfiguration(int t){
  self->kin->K.set()->setFrameState(self->configurations(t+self->k_order)->getFrameState());
  self->kin->K.set()->proxies = self->configurations(t+self->k_order)->proxies;
  for(auto& d:self->kin->cameras) d->gl.update(STRING("KOMOpy configuration " <<t));
}
