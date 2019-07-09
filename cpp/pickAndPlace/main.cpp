#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>
#include <Kin/proxy.h>

#include <LGP/LGP_tree.h>
#include <KOMO/komo.h>


void generateProblem(rai::KinematicWorld& K){
  uint numObj = 4;
  for(;;){
    K.clear();
    K.addFile("../../rai-robotModels/pr2/pr2.g");
    K["pr2L"]->ats.newNode<Graph>({"logical"}, {}, {{"gripper", true}});
    K["pr2R"]->ats.newNode<Graph>({"logical"}, {}, {{"gripper", true}});
    K["worldTranslationRotation"]->joint->H = 1e-0;
    K.addFile("../../models/tables.g");
    for(uint i=0;i<numObj;i++){
      rai::Frame *f = K.addFrame(STRING("obj"<<i), "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid" );
      f->Q.pos = {rnd.uni(-.3, .3), rnd.uni(-1.,1.), .15};
      f->Q.rot.addZ(rnd.uni(-RAI_PI,RAI_PI));
      f->X = f->parent->X * f->Q;
    }
    K.stepSwift();
    arr y;
    K.kinematicsProxyCost(y, NoArr);
//    cout <<"collision costs of config: " <<y.scalar() <<endl;
//    K.reportProxies(cout, .1, true);
//    K.watch();
    if(y.scalar()==0.) break;
  }

  K.proxies.clear();

  rai::Frame *f = K.addFrame("tray", "table2", "type:ssBox size:[.15 .15 .04 .02] color:[0. 1. 0.], logical={ table }" );
 f->Q.pos = {0.,0.,.07};
//  f->Q.pos = {rnd.uni(-.3, .3), rnd.uni(-1.,1.), .07};
//  f->Q.rot.addZ(rnd.uni(-RAI_PI,RAI_PI));

  K.addFrame("", "tray", "type:ssBox size:[.27 .27 .04 .02] color:[0. 1. 0.]" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<t(+.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<t(-.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<d(90 0 0 1) t(+.13 0 .03)> color:[0. 1. 0.], contact" );
//  K.addFrame("", "tray", "type:ssBox size:[.04 .3 .1 .02] Q:<d(90 0 0 1) t(-.13 0 .03)> color:[0. 1. 0.], contact" );

  K.calc_fwdPropagateFrames();
}


void solve(){
  rai::KinematicWorld K;
  generateProblem(K);
  //  K.addFile("model2.g");
  K.selectJointsByGroup({"base","armL","armR"}, true, true);
  K.optimizeTree();

  LGP_Tree lgp(K, "../../models/fol-pickAndPlace.g");
  lgp.fol.addTerminalRule("(on obj0 tray) (on obj1 tray) (on obj2 tray)");
  lgp.displayBound = BD_seqPath;
  lgp.verbose=2;


  lgp.run();

//  lgp.optFixedSequence("(grasp pr2R obj1) (place pr2R obj1 tray)", 2, true);
//  lgp.optFixedSequence("(grasp pr2R obj0) (grasp pr2L obj3) (place pr2R obj0 tray) ", BD_seq, true);
//  lgp.optFixedSequence("(grasp pr2R obj0) (grasp pr2L obj3) (place pr2R obj0 tray) ", BD_seqPath, true);
//  lgp.optFixedSequence("(grasp pr2R obj3)", 2, true);
//  lgp.optFixedSequence("(grasp pr2R obj1) (grasp pr2L obj2) (place pr2R obj1 tray) (place pr2L obj2 tray)\
//                       (grasp pr2R obj3) (grasp pr2L obj0) (place pr2R obj3 tray) (place pr2L obj0 tray)\
//                       ", 2, true);

  rai::wait();
  lgp.renderToVideo();
}

void testBounds(){
  rai::KinematicWorld K;
  generateProblem(K);
//  K.addFile("model2.g");
  K.selectJointsByGroup({"base","armL","armR"}, true, true);
  K.optimizeTree();

  LGP_Tree lgp(K, "../../models/fol-pickAndPlace.g");

//  lgp.getSymbolicSolutions(6);
//  return;

  LGP_Node *node = lgp.walkToNode("(grasp pr2R obj0) (grasp pr2L obj3) (place pr2R obj0 tray)");
  cout <<"Node Info:\n" <<node->getInfo() <<endl;

  auto S = node->getSkeleton();
  writeSkeleton(cout, S, getSwitchesFromSkeleton(S));

//  node = node->parent->parent;
//  node->computeEndKinematics();
//  node->effKinematics.glAnimate();

  BoundType bound = BD_pose;
//  node->optBound(bound, true);
//  node->effKinematics.glAnimate();
//  node->komoProblem(bound)->displayTrajectory(-1., true, false);

  bound = BD_seq;
  node->optBound(bound, true, 2);
  node->komoProblem(bound)->displayTrajectory(-1., true, false);

  bound = BD_seqPath;
  node->optBound(bound, true, 2);
  node->komoProblem(bound)->displayTrajectory(.1, true, false);

  cout <<"Node Info:\n" <<node->getInfo() <<endl;
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  //rnd.clockSeed();

  solve();
  //testBounds();

  return 0;
}
