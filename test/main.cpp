#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/TM_ContactConstraints.h>
//#include <KOMOcsail/komo-CSAIL.h>
#include <Kin/TM_default.h>

#include <ry/configuration.h>


//===========================================================================

void test(){

  auto K = ry::Configuration();
  auto D = K.camera();
  D.update("empty configuration\n -- hit ENTER here to continue", true);

  K.addFile("../rai-robotModels/pr2/pr2.g");
  K.addFile("kitchen.g");
  cout <<"joint names: " << I_conv(K.getJointNames()) <<endl;
  cout <<"frame names: " << I_conv(K.getFrameNames()) <<endl;
  D.update(true);

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

  K.addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(.8 .8 1.5)>" );
  D.update(true);

  K.addFrame("hand", "pr2L", "shape:ssBox size:[.3 .2 .1 .01] color:[1 1 0] Q:<t(0 0 0)>" );
  D.update(true);

  K.getPairDistance("hand", "ball");
  D.update(true);

  K.stash();
  {
    auto komo = K.komo_IK();
    komo.addObjective({}, {}, "eq", "posDiff", {"pr2L", "ball"});
//    komo.addObjectives2( { "feature:[eq posDiff pr2L ball]" } );
    komo.optimize();
    D.update(true);

    komo.clearObjectives();
    komo.addObjective({}, {}, "eq", "posDiff", {"pr2L", "ball"}, {}, {.1,.1,.1});
    komo.optimize();
  }
  D.update(true);

  K.pop();
  {
    auto komo = K.komo_path(1.);
//    komo.addObjectives2( { "time:[1.], feature:[eq posDiff pr2L ball]",
//                          "time:[1.], feature:[eq qRobot], order:1",
//                        } );
    komo.addObjectives({   ry::I_feature({1.}, {"eq", "posDiff", "pr2L", "ball"}, ry::I_args() ),
	  ry::I_feature({1.}, {"eq", "qRobot"}, {{std::string("order"), {1.}}} )
                       });
    komo.optimize();
  }
  D.update(true);

}

//===========================================================================

void test_camera(){

  auto K = ry::Configuration();
  auto D = K.camera();
  D.update("empty configuration\n -- hit ENTER here to continue", true);

  K.addFile("../rai-robotModels/pr2/pr2.g");
  K.addFile("kitchen.g");
  D.update(true);

  K.addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.5");
  auto C = K.camera("camera");
  C.update(true);

  K.setJointState({1.}, {"head_pan_joint"});
  C.update(false);
  D.update(true);

}

//===========================================================================

void test_constraints(){
  auto K = ry::Configuration();
  auto D = K.camera();
  K.addFile("../rai-robotModels/pr2/pr2.g");
  K.addFile("../test/kitchen.g");
//  auto x0 = K.getFrameState();
  K.addFrame("goal", "", "shape:marker size:[.3] color:[.5 1 1]" );
  K.setFrameState({1,1,1,1,0,0,0}, {"goal"});
  D.update(true);
}

//===========================================================================

void test_pickAndPlace(){
  auto K = ry::Configuration();
  auto D = K.camera();

  K.addFile("../rai-robotModels/pr2/pr2.g");
  K.addFile("kitchen.g");

  K.addFrame("item1", "sink1", "type:ssBox Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact" );
  K.addFrame("item2", "sink1", "type:ssBox Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact" );
  K.addFrame("tray", "stove1", "type:ssBox Q:<t(.0 .0 .42)> size:[.2 .2 .05 .02] color:[0. 1. 0.], contact" );

  auto obj1 = "item2";
  auto obj2 = "item1";
  auto tray = "tray";
  auto arm = "pr2L";
  auto table = "_12";

  int T=6;
  auto komo = K.komo_CGO(T);

  komo.activateCollisionPairs({{obj1, obj2}});
  komo.addObjective({}, {}, "eq", "coll");
  komo.addObjective({}, {}, "ineq", "limits");

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
    D.update(true);
  }
}

//===========================================================================

void test_skeleton(){

  auto K = ry::Configuration();
  auto D = K.camera();

  K.addFile("lgp-example.g");
  D.update();

  auto komo = K.komo_path(1.);

  //we're creating the same skeleton that'd be created by the decision sequence
  //(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)
  //which is a standard demo in the RSS'18 paper

  //(grasp baxterR stick)
  komo.addSkeleton({1,1}, {"touch", "baxterR", "stick"} );
  komo.addSkeleton({1,1}, {"stable", "baxterR", "stick"} );
  komo.addSkeleton({1,1}, {"liftDownUp", "baxterR"} );

  //(handover baxterR stick baxterL)
  komo.addSkeleton({2,2}, {"touch", "baxterL", "stick"} );
  komo.addSkeleton({2,4}, {"stable", "baxterL", "stick"} );

  //(hitSlide stickTip redBall table1)
  komo.addSkeleton({3,3}, {"touch", "stickTip", "redBall"} );
  komo.addSkeleton({3,3}, {"impulse", "stickTip", "redBall"} );
  komo.addSkeleton({3,3}, {"dynamicOn", "table1", "redBall"} );

  //(graspSlide baxterR redBall table1)
  komo.addSkeleton({4,4}, {"graspSlide", "baxterR", "redBall", "table1"} );

  komo.skeleton2bound();

  komo.optimize();

  for(int t=-1;t<komo.getT();t++){
    komo.getConfiguration(t);
    D.update(true);
  }
}

//===========================================================================

void test_skeleton2(){

  auto K = ry::Configuration();
  auto D = K.camera();

  K.addFile("boxProblem.g");
  D.update();

  auto komo = K.komo_path(1., 50, 2.);

  //-- this is all yet 'magic' -> clearer interface
  komo.timeOptimization();
  komo.deactivateCollisionPairs({{"boxBo", "boxLe"}, {"boxBo", "boxBa"}, {"boxLe", "boxBa"}});
  komo.makeObjectsFree({"ballR"});
  komo.addObjective({}, {.05, -1.}, "eq", "physics", {"ballR"}, {1e-1});
  komo.addObjective({}, {.05, -1.}, "ineq", "energy", {}, {1e-1});
  komo.addObjective({}, {}, "sos", "accumulatedCollisions", {}, {1.});
  komo.addObjective({}, {}, "eq", "contactConstraints", {}, {3e1});

  //-- this is the skeleton
  komo.addSkeleton({.4, .4}, {"contact", "boxBo", "ballR"} );
  komo.addSkeleton({.6, .6}, {"contact", "boxBo", "ballR"} );
  komo.addSkeleton({.8, .8}, {"contact", "boxBo", "ballR"} );
  komo.addSkeleton({1., 1.}, {"touch", "target", "ballR"} );
  komo.setSkeleton();

  komo.optimize();

  for(int t=-1;t<komo.getT();t++){
    komo.getConfiguration(t);
    D.update(true);
  }
}

//===========================================================================

void test_lgp(){
  auto K = ry::Configuration();
  auto D = K.camera();

  K.addFile("lgp-example.g");
  D.update(true);

  auto lgp = K.lgp("fol.g");

  lgp.optimizeFixedSequence("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  test();
//  test_camera();
//  test_pickAndPlace();
//  test_constraints();
//  test_skeleton();
  test_skeleton2();
//  test_lgp();

  return 0;
}


