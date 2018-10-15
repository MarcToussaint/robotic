#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>
//#include <Kin/TM_ContactConstraints.h>
//#include <KOMOcsail/komo-CSAIL.h>
//#include <Kin/TM_default.h>
//#include <Kin/TM_linTrans.h>

#include <Operate/path.h>

#include <ry/pybind.h>
#include <Operate/robotio.h>

//===========================================================================

void test(){

  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("../rai-robotModels/pr2/pr2.g");
  K.set()->addFile("kitchen.g");
  cout <<"joint names: " << K.get()->getJointNames() <<endl;
  cout <<"frame names: " << K.get()->getFrameNames() <<endl;

  auto q = K.get()->getJointState();

  //  K.editorFile("../rai-robotModels/baxter/baxter.g");

  //    q = K.getJointState()
  //    print('joint state: ', q)
  //    q[2] = q[2] + 1.
  //    K.setJointState(q)
  //    D.update(True)

  //    X = K.getFrameState()
  //    print('frame state: ', X)
  //    X = X + .1
  //    K.setFrameState(X)
  //    D.update(True)

  //    q = K.getJointState()
  //    print('joint state: ', q)
  //    q[2] = q[2] + 1.
  //    K.setJointState(q)
  //    D.update(True)

//  K.addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.5");
//  auto C = K.camera("camera");

  K.set()->addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(.8 .8 1.5)>" );

  K.set()->addFrame("hand", "pr2L", "shape:ssBox size:[.3 .2 .1 .01] color:[1 1 0] Q:<t(0 0 0)>" );

  {
    auto komo = ry::KOMOpy(K,0);
    komo.addObjective2({}, OT_eq, FS_positionDiff, {"pr2L", "ball"});
//    komo.addObjectives2( { "feature:[eq posDiff pr2L ball]" } );
    komo.optimize();
    komo.getConfiguration(0);
    rai::wait();

    komo.clearObjectives();
    komo.addObjective2({}, OT_eq, FS_positionDiff, {"pr2L", "ball"}, {}, {.1,.1,.1});
    komo.optimize();
    komo.getConfiguration(0);
    rai::wait();
  }

  K.set()->setJointState(q);
  {
    auto komo = ry::KOMOpy(K, 1., 20, 5.);
//    komo.addObjectives2( { "time:[1.], feature:[eq posDiff pr2L ball]",
//                          "time:[1.], feature:[eq qRobot], order:1",
//                        } );
    komo.addObjective({1.}, OT_eq, FS_positionDiff, {"pr2L", "ball"});
    komo.addObjective({1.}, OT_eq, FS_qItself, {}, {}, {}, 1); //zero q-velocity at goal

    komo.optimize();

    for(int t=-1;t<20;t++){
      komo.getConfiguration(t);
      rai::wait();
    }
  }
//  D.update(true);

}

//===========================================================================

void test_camera(){

  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();
  rai::wait();

  K.set()->addFile("../rai-robotModels/pr2/pr2.g");
  K.set()->addFile("kitchen.g");
  rai::wait();

  K.set()->addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.5 width:300 height:200");
  auto C = KinViewer(K, -1., "camera"); //K.camera("camera");
  rai::wait();

  K.set()->setJointState({1.}, {"head_pan_joint"});
  rai::wait();

}

//===========================================================================

void test_constraints(){
  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();
  K.set()->addFile("../rai-robotModels/pr2/pr2.g");
  K.set()->addFile("../test/kitchen.g");
  K.set()->addObject("goal", rai::ST_sphere, {.1}, {.5, 1., 1.});
  K.set()->setFrameState({1,1,1,1,0,0,0}, {"goal"});
  rai::wait();
}

//===========================================================================

void test_pickAndPlace(){
  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("../rai-robotModels/pr2/pr2.g");
  K.set()->addFile("kitchen.g");

  K.set()->addFrame("item1", "sink1", "type:ssBox Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact" );
  K.set()->addFrame("item2", "sink1", "type:ssBox Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact" );
  K.set()->addFrame("tray", "stove1", "type:ssBox Q:<t(.0 .0 .42)> size:[.2 .2 .05 .02] color:[0. 1. 0.], contact" );

  auto obj1 = "item2";
  auto obj2 = "item1";
  auto tray = "tray";
  auto arm = "pr2L";
  auto table = "_12";

  int T=6;
  auto komo = ry::KOMOpy(K, T);

  komo.activateCollisionPairs({{obj1, obj2}});
  komo.addObjective2({}, OT_eq, FS_accumulatedCollisions);
  komo.addObjective2({}, OT_ineq, FS_jointLimits);

  komo.add_StableRelativePose({0, 1}, arm, obj1);
  komo.add_StableRelativePose({2, 3}, arm, obj2);
  komo.add_StableRelativePose({4, 5}, arm, tray);

  komo.add_StableRelativePose({1,2,3,4,5}, tray, obj1);
  komo.add_StableRelativePose({3,4,5}, tray, obj2);

  komo.add_StablePose({-1,0}, obj1);
  komo.add_StablePose({-1,0,1,2}, obj2);
  komo.add_StablePose({-1,0,1,2,3,4}, tray);

  komo.add_grasp(0, arm, obj1);
  komo.add_place(1, obj1, tray);

  komo.add_grasp(2, arm, obj2);
  komo.add_place(3, obj2, tray);

  komo.add_grasp(4, arm, tray);
  komo.add_place(5, tray, table);

  komo.optimize();

  for(int t=-1;t<T;t++){
    komo.getConfiguration(t);
    rai::wait();
  }
}

//===========================================================================

void test_skeleton(){

  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("lgp-example.g");

  auto komo = ry::KOMOpy(K, 1., 20, 5.);

  //we're creating the same skeleton that'd be created by the decision sequence
  //(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)
  //which is a standard demo in the RSS'18 paper

  //(grasp baxterR stick)
  Skeleton S;
  S.append({1.,1., {"touch", "baxterR", "stick"} });
  S.append({1,1, {"stable", "baxterR", "stick"} });
  S.append({1,1, {"liftDownUp", "baxterR"} });

  //(handover baxterR stick baxterL)
  S.append({2,2, {"touch", "baxterL", "stick"} });
  S.append({2,4, {"stable", "baxterL", "stick"} });

  //(hitSlide stickTip redBall table1)
  S.append({3,3, {"touch", "stickTip", "redBall"} });
  S.append({3,3, {"impulse", "stickTip", "redBall"} });
  S.append({3,3, {"dynamicOn", "table1", "redBall"} });

  //(graspSlide baxterR redBall table1)
  S.append({4,4, {"graspSlide", "baxterR", "redBall", "table1"} });

//  komo.self->setSkeleton(S);
//  komo.self->skeleton2bound();
  skeleton2Bound(*komo.self, BD_path, S, komo.self->world, komo.self->world, false);

  komo.optimize();
  komo.display();

  for(int t=-1;t<komo.getT();t++){
    komo.getConfiguration(t);
    rai::wait();
  }
}

//===========================================================================

void test_skeleton2(){

  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("boxProblem.g");

  auto komo = ry::KOMOpy(K, 1., 50, 2.);

  //-- this is all yet 'magic' -> clearer interface
  komo.timeOptimization();
  komo.deactivateCollisionPairs({{"boxBo", "boxLe"}, {"boxBo", "boxBa"}, {"boxLe", "boxBa"}});
  komo.makeObjectsFree({"ballR"});
  komo.addObjective({.05, -1.}, OT_eq, FS_physics, {"ballR"}, {1e-1});
  komo.addObjective({.05, -1.}, OT_ineq, FS_energy, {}, {1e-1});
  komo.addObjective({}, OT_sos, FS_accumulatedCollisions, {}, {1.});

  //-- this is the skeleton
  Skeleton S;
  S.append({.4, .4, {"contact", "boxBo", "ballR"} });
  S.append({.6, .6, {"contact", "boxBo", "ballR"} });
  S.append({.8, .8, {"contact", "boxBo", "ballR"} });
  S.append({1., 1., {"touch", "target", "ballR"} });
  komo.self->setSkeleton(S, true);

  komo.optimize();

  for(int t=-1;t<komo.getT();t++){
    komo.getConfiguration(t);
    rai::wait();
  }
}

//===========================================================================

void test_lgp(){
  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("lgp-example.g");

  auto lgp = ry::LGPpy(K, "fol.g");

  lgp.optimizeFixedSequence("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");
}

//===========================================================================

std::pair<arr,arr> computePath(ry::Config& K, const arr& target_q, const StringA& target_joints, const char* endeff, double up, double down){
  KOMO komo;
  komo.setModel(K.get(), true, true);
  komo.setPathOpt(1., 20, 3.);

  addMotionTo(komo, target_q, target_joints, endeff, up, down);
  komo.optimize();

  arr path = komo.getPath(target_joints);
  path[path.d0-1] = target_q; //overwrite last config
  arr tau = komo.getPath_times();
  cout <<validatePath(K.get(), K->getJointState(), target_joints, path, tau) <<endl;
  bool go = komo.displayPath(true);//;/komo.display();
  if(!go){
    cout <<"ABORT!" <<endl;
    return {arr(), arr()};
  }
  return {path, tau};
}

void test_realGrasp(){
  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("../rai-robotModels/pr2/pr2.g");
  K.set()->addFile("kitchen.g");

  K.set()->addFrame("item1", "sink1", "type:ssBox Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact, joint:rigid" );
  K.set()->addFrame("item2", "sink1", "type:ssBox Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact" );
  K.set()->addFrame("tray", "stove1", "type:ssBox Q:<t(.0 .0 .42)> size:[.2 .2 .05 .02] color:[0. 1. 0.], contact" );

  RobotIO R(K.get(), ROB_sim);

  const char* endeff="pr2R";
  const char* object="item1";

  arr s0 = K->getFrameState();

  K.set()->selectJointsByGroup({"armR","base"});
  StringA armBase = K->getJointNames();

  chooseBoxGrasp(K.set(), endeff, object);
  arr grasp = K->getJointState();
  rai::wait();

  K->setFrameState(s0);

  auto path = computePath(K, grasp, armBase, endeff, .0, .8);

  R.execGripper("pr2R", .1);
  R.waitForCompletion();

  R.executeMotion(armBase, path.first, path.second, .5);
  R.waitForCompletion();

  R.execGripper("pr2R", .0);
  R.waitForCompletion();

  R.attach(endeff, object);

  arr q_now = R.getJointPositions(armBase);
  K.set()->setJointState(q_now);
  rai::wait();

  path = computePath(K, zeros(grasp.N), armBase, endeff, .2, .8);
  R.executeMotion(armBase, path.first, path.second, .5);
  R.waitForCompletion();


  rai::wait();

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  test();
//  test_camera();
//  test_pickAndPlace();
//  test_constraints();
  test_skeleton();

//  test_skeleton2();
//  test_lgp();

//  test_realGrasp();

  return 0;
}


