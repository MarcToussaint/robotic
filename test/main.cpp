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
  auto D = K.display();
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

  K.addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.5");
  auto C = K.camera("camera");

  K.addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(.8 .8 1.5)>" );
  D.update(true);

  K.addFrame("hand", "pr2L", "shape:ssBox size:[.3 .2 .1 .01] color:[1 1 0] Q:<t(0 0 0)>" );
  D.update(true);

  K.getPairDistance("hand", "ball");
  D.update(true);

  auto komo = K.komo();
  komo.optimize( { I_feature("eq", {"posDiff", "pr2L", "ball"}, {} ) } );
  D.update(true);

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  test();

  return 0;
}


