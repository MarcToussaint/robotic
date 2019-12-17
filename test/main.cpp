#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>
//#include <Kin/TM_ContactConstraints.h>
//#include <KOMOcsail/komo-CSAIL.h>
//#include <Kin/TM_default.h>
//#include <Kin/TM_linTrans.h>
#include <LGP/bounds.h>
#include <Kin/kin_bullet.h>
#include <Kin/kin_physx.h>
#include <Kin/kinViewer.h>
#include <LGP/LGP_tree.h>
#include <RosCom/rosCamera.h>

#include <Operate/path.h>

#include <ry/ry.h>
#include <Operate/robotio.h>

void miniTest(){
  rai::Configuration C;
  C.addFile("../rai-robotModels/pr2/pr2.g");
  C.addFile("kitchen.g");
  C.watch(true);

  arr X0 = C.getFrameState();
  arr X1 = X0 + .1;

  cout <<X1[0] <<endl;

  C.setFrameState(X1);
  C.watch(true);

  arr X2 = C.getFrameState();

  cout <<X2[0] <<endl;
  cout <<X1[0] - X2[0] <<endl;

  C.setFrameState(X0);
  C.watch(true);
}


//===========================================================================

void test(){
  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();

  C.set()->addFile("../rai-robotModels/pr2/pr2.g");
  C.set()->addFile("kitchen.g");
  cout <<"joint names: " << C.get()->getJointNames() <<endl;
  cout <<"frame names: " << C.get()->getFrameNames() <<endl;
  rai::wait();

  auto ball = C.set()->addFrame("ball");
  ball->setShape(rai::ST_sphere, {.1});
  ball->setPosition({.8,.8,1.5});
  ball->setColor({1,1,0});
  C.set(); //forces the display to update...
  rai::wait();

  auto hand = C.set()->addFrame("hand", "pr2L");
  hand->setShape(rai::ST_ssBox, {.2,.2,.1,.02});
  hand->setRelativePosition({0,0,-.1});
  hand->setColor({1,1,0});
  C.set();
  rai::wait();

  auto q0 = C.get()->getJointState();
  cout <<"joint names: " <<C.get()->getJointNames() <<endl;
  cout <<"joint state: " <<q0 <<endl;
  rai::wait();

  q0(2) = q0(2) + 1.;
  C.set()->setJointState(q0);
  rai::wait();

  FILE("z.kin1") <<C.get()() <<endl;

  auto X0 = C.get()->getFrameState();
  cout <<"frame state: " <<X0 <<endl;
  rai::wait();

  auto X = X0 + .1;
  C.set()->setFrameState(X);
  auto X2 = C.get()->getFrameState();
  cout <<X - X2 <<endl;
  rai::wait();

  auto q = C.get()->getJointState();
  cout <<"q: " <<q <<endl;
  C.set()->setJointState(q);
  rai::wait();

  C.set()->setFrameState(X0);
  rai::wait();

  FILE("z.kin2") <<C.get()() <<endl;

  q = C.get()->getJointState();
  cout <<"q0: " <<q0 <<endl;
  cout <<"q: " <<q <<endl;
  C.set()->setJointState(q);
  rai::wait();

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


  {
    auto komo = ry::RyKOMO(C,0);
    komo.komo->addObjective({}, FS_positionDiff, {"pr2L", "ball"}, OT_eq);
//    komo.addObjectives2( { "feature:[eq posDiff pr2L ball]" } );
    komo.komo->optimize();
    komo.komo->getConfiguration(0);
    rai::wait();

    komo.komo->clearObjectives();
    komo.komo->addObjective({}, FS_positionDiff, {"pr2L", "ball"}, OT_eq, {}, {.1,.1,.1});
    komo.komo->optimize();
    komo.komo->getConfiguration(0);
    rai::wait();
  }

  C.set()->setJointState(q0);
  {
    auto komo = ry::RyKOMO(C, 1., 20, 5., false);
//    komo.addObjectives2( { "time:[1.], feature:[eq posDiff pr2L ball]",
//                          "time:[1.], feature:[eq qRobot], order:1",
//                        } );
    komo.komo->addObjective({1.}, FS_positionDiff, {"pr2L", "ball"}, OT_eq);
    komo.komo->addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1); //zero q-velocity at goal

    komo.komo->optimize();

    for(int t=-1;t<20;t++){
      komo.komo->getConfiguration(t);
      rai::wait();
    }
  }
//  D.update(true);

}

//===========================================================================

void test_camera(){

  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();
  rai::wait();

  C.set()->addFile("../rai-robotModels/pr2/pr2.g");
  C.set()->addFile("kitchen.g");
  rai::wait();

  C.set()->addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.5 width:300 height:200");
  auto V = KinViewer(C, -1., "camera"); //K.camera("camera");
  rai::wait();

  C.set()->setJointState({1.}, {"head_pan_joint"});
  rai::wait();
}

//===========================================================================

void test_constraints(){
  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();
  C.set()->addFile("../rai-robotModels/pr2/pr2.g");
  C.set()->addFile("../test/kitchen.g");
  C.set()->addObject("goal", NULL, rai::ST_sphere, {.1}, {.5, 1., 1.}, {1.,1.,1.});
  rai::wait();
}

//===========================================================================

void test_pickAndPlace(){
  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();

  C.set()->addFile("../rai-robotModels/pr2/pr2.g");
  C.set()->addFile("kitchen.g");

  C.set()->addObject("item1", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 0., 0.}, {-.1, -.1, .52});
  C.set()->addObject("item2", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 1., 0.}, {.1, .1, .52});
  C.set()->addObject("tray", "stove1", rai::ST_ssBox, {.2, .2, .05, .02}, {0., 1., 0.}, {.0, .0, .42});

  auto obj1 = "item2";
  auto obj2 = "item1";
  auto tray = "tray";
  auto arm = "pr2L";
  auto table = "_12";

  KOMO komo(C.get());
  komo.setDiscreteOpt(6);

  komo.activateCollisions(obj1, obj2);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq);
  komo.addObjective({}, FS_jointLimits, {}, OT_ineq);

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

  komo.displayPath();
  komo.displayTrajectory(1., true, false);
}

//===========================================================================

void test_path(){
  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();

  C.set()->addFile("../rai-robotModels/pr2/pr2.g");
  C.set()->addFile("kitchen.g");

  C.set()->addObject("item1", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 0., 0.}, {-.1, -.1, .52});
  C.set()->addObject("item2", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 1., 0.}, {.1, .1, .52});
  C.set()->addObject("tray", "stove1", rai::ST_ssBox, {.2, .2, .05, .02}, {0., 1., 0.}, {.0, .0, .42});

  auto obj1 = "item1";
  auto arm = "pr2R";

  KOMO komo(C.get());
  komo.setPathOpt(1.,20, 10.);
  komo.setSquaredQAccVelHoming();

//  komo.addObjective({}, OT_sos, FS_transAccelerations, {}, {1.});
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq);
  komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
  komo.addObjective({1.}, FS_distance, {arm, obj1}, OT_eq);
  komo.addObjective({.9,1.}, FS_positionDiff, {"endeffWorkspace", obj1}, OT_sos, {1e0});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);


  komo.optimize();

  Var<arr> path;
  path.set() = komo.getPath_frames();
  KinPoseViewer komoV(C, path, .1);
  rai::wait();

  komo.displayPath();
  komo.displayTrajectory(1., true, false);
}

//===========================================================================

void test_skeleton(){

  auto C = ry::Config();
  auto D = KinViewer(C); //K.view();

  C.set()->addFile("lgp-example.g");

  auto komo = ry::RyKOMO(C, 1., 20, 5., false);

  //we're creating the same skeleton that'd be created by the decision sequence
  //(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)
  //which is a standard demo in the RSS'18 paper

  //(grasp baxterR stick)
  Skeleton S;
  S.append({1,1, SY_touch, {"baxterR", "stick"} });
  S.append({1,1, SY_stable, {"baxterR", "stick"} });
  S.append({1,1, SY_liftDownUp, {"baxterR"} });

  //(handover baxterR stick baxterL)
  S.append({2,2, SY_touch, {"baxterL", "stick"} });
  S.append({2,4, SY_stable, {"baxterL", "stick"} });

  //(hitSlide stickTip redBall table1)
  S.append({3,3, SY_touch, {"stickTip", "redBall"} });
  S.append({3,3, SY_impulse, {"stickTip", "redBall"} });
  S.append({3,3, SY_dynamicOn, {"table1", "redBall"} });

  //(graspSlide baxterR redBall table1)
  S.append({4,4, SY_graspSlide, {"baxterR", "redBall", "table1"} });

//  komo.self->setSkeleton(S);
//  komo.self->skeleton2bound();
  skeleton2Bound(*komo.komo, BD_path, S, komo.komo->world, komo.komo->world, false);

  auto view = KinPoseViewer(komo.config, komo.path, .1);
  komo.komo->optimize();
  komo.path.set() = komo.komo->getPath_frames();
  komo.komo->displayTrajectory();

  for(int t=-1;t<(int)komo.komo->T;t++){
    komo.komo->getConfiguration(t);
    rai::wait();
  }
}

//===========================================================================

void test_lgp(){
  auto K = ry::Config();
  auto D = KinViewer(K); //K.view();

  K.set()->addFile("lgp-example.g");
  makeConvexHulls(K.set()->frames, true);
//  K.set()->stepSwift();
//  cout <<"TOTAL INITIAL PENETRATION: " <<K.set()->totalContactPenetration() <<endl;
//  K.set()->reportProxies();

  LGP_Tree_Thread lgp(K.set(), "fol.g");

  lgp.walkToNode("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");
//  lgp.walkToNode("(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)");

  lgp.focusNode->optBound(BD_path, true, 2);
  lgp.focusNode->komoProblem(BD_path)->displayTrajectory(.02, false, false);

  rai::wait();
}

//===========================================================================

void test_skeleton2(){

  auto K = ry::Config();
  auto D = KinViewer(K); //K`view();

  K.set()->addFile("boxProblem.g");

  auto komo = ry::RyKOMO(K, 1., 20, 5., false);

  //-- this is all yet 'magic' -> clearer interface
  komo.komo->setTimeOptimization();
//  komo.komo->deactivateCollisions({{"boxBo", "boxLe"}, {"boxBo", "boxBa"}, {"boxLe", "boxBa"}});
  komo.komo->world.makeObjectsFree({"ballR"});
  komo.komo->addObjective({.05, -1.}, FS_physics, {"ballR"}, OT_eq, {1e-1});
  komo.komo->addObjective({.05, -1.}, FS_energy, {}, OT_ineq, {1e-1});
  komo.komo->addObjective({}, FS_accumulatedCollisions, {}, OT_sos, {1.});

  //-- this is the skeleton
  Skeleton S;
  S.append({.4, .4, SY_contact, {"boxBo", "ballR"} });
  S.append({.6, .6, SY_contact, {"boxBo", "ballR"} });
  S.append({.8, .8, SY_contact, {"boxBo", "ballR"} });
  S.append({1., 1., SY_touch, {"target", "ballR"} });
  komo.komo->setSkeleton(S, true);

  komo.komo->optimize();

  for(int t=-1;t<(int)komo.komo->T;t++){
    komo.komo->getConfiguration(t);
    rai::wait();
  }
}

//===========================================================================

#if 0
std::pair<arr,arr> computePath(ry::Config& K, const arr& target_q, const StringA& target_joints, const char* endeff, double up, double down){
  KOMO komo;
  komo.setModel(K.get(), true);
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

  C.set()->addObject("item1", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 0., 0.}, {-.1, -.1, .52});
  C.set()->addObject("item2", "sink1", rai::ST_ssBox, {.1, .1, .25, .02}, {1., 1., 0.}, {.1, .1, .52});
  C.set()->addObject("tray", "stove1", rai::ST_ssBox, {.2, .2, .05, .02}, {0., 1., 0.}, {.0, .0, .42});

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
#endif

//===========================================================================

void test_bullet(){

  auto C = ry::Config();
  auto D = KinViewer(C);

  C.set()->addFrame("world");
  C.set()->addObject("block", NULL, rai::ST_ssBox, {.2,.3,.5,.02}, {1,0,0}, {.0,.0,.5});
  auto *finger = C.set()->addObject("finger", "world", rai::ST_ssBox, {.3, .1, .1, .02}, {1.,1.,1.,.3}, {1., 0., .45});

  rai::wait();

  auto B = PhysXInterface(C.get());
  arr x = finger->getPosition();
  arr V,X;

  for(int i=0;i<100;i++){
    x(0) -= .01;
    finger->setPosition(x);
    B.pushKinematicStates(C.get()->frames);
    B.step();
    B.pullDynamicStates(C.set()->frames, V);

    rai::wait(0.01);
  }

  X = C.get()->getFrameState();

  for(uint k=0;k<3;k++){
    C.set()->setFrameState(X);
    x = finger->getPosition();
    B.pushFullState(C.get()->frames, V);

    for(int i=0;i<100;i++){
      x(0) -= .01;
      finger->setPosition(x);
      B.pushKinematicStates(C.get()->frames);
      B.step();
      B.pullDynamicStates(C.set()->frames, V);

      rai::wait(0.01);
    }
  }
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  miniTest();
  test();
//  test_camera();
//  test_pickAndPlace();
//  test_path();
//  test_constraints();

//  test_skeleton();

//  test_skeleton2();
//  test_lgp();

//  test_realGrasp();

//  test_bullet();

  return 0;
}


