#include "komo-py.h"
#include "configuration.h"



#include "komo-py.h"

#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>
#include <Kin/TM_qItself.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/TM_transition.h>
#include <Kin/TM_qLimits.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);

ry::KOMOpy_self::KOMOpy_self(ry::Configuration* _kin) : kin(_kin) {
//  LOG(0) <<"create " <<this;
  setModel(kin->K.get(), false);
  setIKOpt();

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
Task *ry::KOMOpy_self::setObjective(const intA& vars, ObjectiveType type, TaskMap *map, const arr &target, double scale){
  uint order = vars.N-1;
  CHECK_GE(k_order, order, "task requires larger k-order: " <<map->shortTag(world));
  map->order = order;
  Task *task = addTask(map->shortTag(world), map, type);
//  if(vars.N==2) CHECK_EQ(vars(1), vars(0)+1, "so far only consecutive vars are supported");
//  if(vars.N==3){
//    CHECK_EQ(vars(1), vars(0)+1, "so far only consecutive vars are supported");
//    CHECK_EQ(vars(2), vars(1)+1, "so far only consecutive vars are supported");
//  }
  task->setCostSpecs(vars(-1), vars(-1), target, scale);
//  task->setCostSpecsDense(vars, target, scale);
  return task;
}

TaskMap *ry::KOMOpy_self::symbols2feature(const StringA &symbols, const std::map<std::string, arr>& parameters){
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

  double margin=.05;
  if(parameters.find("margin")!=parameters.end()) margin = parameters.at("v1").scalar();

  if(symbols(0)=="coll") {  return new TM_Proxy(TMT_allP, {0u}, margin, true); }
  if(symbols(0)=="limits") {  return new LimitsConstraint(margin); }

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
void ry::KOMOpy_self::setObjective(const intA& vars, ObjectiveType type, const StringA &featureSymbols, const std::map<std::string, arr> &parameters){
  arr target;
  double scale=1e1;
  if(parameters.find("scale")!=parameters.end()) scale = parameters.at("scale").scalar();
  if(parameters.find("target")!=parameters.end()) target = parameters.at("target");

  setObjective(vars, type, symbols2feature(featureSymbols, parameters), target, scale);
}


ry::KOMOpy::KOMOpy(ry::Configuration* _kin)
  : self(make_shared<ry::KOMOpy_self>(_kin)) { //self(make_shared<KinIK_self>(K)){
}

ry::KOMOpy::~KOMOpy(){
}

void ry::KOMOpy::optimize(const I_features& features){
  for (const I_feature& feature : features) {
    rai::Enum<ObjectiveType> type;
    rai::String( std::get<0>(feature) ) >>type;

    StringA symbols = I_conv(std::get<1>(feature));

    std::map<std::string, arr> parameters;
    for (const auto& x : std::get<2>(feature)) {
      parameters.insert(std::make_pair(x.first, conv_stdvec2arr(x.second)));
    }

    self->setObjective({0}, type, symbols, parameters);
  }

  self->reset();
  self->reportProblem();
  Graph specs = self->getProblemGraph();
  cout <<specs <<endl;

  self->run(false);

  specs = self->getProblemGraph();
  cout <<specs <<endl;
  cout <<self->getReport(false) <<endl; // Enables plot

  self->kin->K.set()->setJointState(self->x);
  for(auto& d:self->kin->displays) d->gl.update(STRING("KOMOpy::optimization end pose"));
}
