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
    komo.addObjectives2( { "time:[1.], feature:[eq posDiff pr2L ball]",
                          "time:[1.], feature:[eq qRobot], order:1",
                        } );
//    komo.optimize( {   I_feature({1.}, {"eq", "posDiff", "pr2L", "ball"}, I_args() ),
//                       I_feature({1.}, {"eq", "qRobot"}, {{std::string("order"), {1.}}} )
//                   } );
    komo.optimize();
  }
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

  auto obj1 = "item2";
  auto obj2 = "item1";
  auto arm = "pr2L";
  auto table = "_13";
  int c0=0, c1=1, c2=2, c3=3;

  auto komo = K.komo_CGO(4);

  komo.add_GraspDecisionVariable({c0, c1}, arm, obj1);
  komo.add_GraspDecisionVariable({c2, c3}, arm, obj2);
  komo.add_PoseDecisionVariable({-1, c0}, obj1);
  komo.add_PoseDecisionVariable({c1, c2, c3}, obj1);
  komo.add_PoseDecisionVariable({-1, c0, c1, c2}, obj2);

  komo.add_IsGraspKin(c0, arm, obj1);
  komo.add_IsPlaceKin(c1, obj1, table);
  komo.add_IsGraspKin(c2, arm, obj2);
  komo.add_IsPlaceKin(c3, obj2, table);

  komo.optimize();

  komo.getConfiguration(-1); D.update(true);
  komo.getConfiguration(0); D.update(true);
  komo.getConfiguration(1); D.update(true);
  komo.getConfiguration(2); D.update(true);
  komo.getConfiguration(3); D.update(true);
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  test();
  test_pickAndPlace();

  return 0;
}


