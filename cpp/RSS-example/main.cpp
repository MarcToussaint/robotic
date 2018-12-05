#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

#include <LGP/LGP_tree.h>
#include <KOMO/komo.h>

void solve(uint i){
  rai::KinematicWorld K(STRING("../../models/RSSproblem-0"<<i<<".g"));
  K.optimizeTree();
  FOL_World L(FILE("../../models/RSSfol.g"));
  initFolStateFromKin(L, K);

  LGP_Tree lgp(K, L);
//  lgp.buildTree(2);  rai::wait();
  lgp.run();
}


void solve1(){
  rai::KinematicWorld K("../../models/RSSproblem-01.g");
  K.optimizeTree();
  FOL_World L(FILE("../../models/RSSfol.g"));
  initFolStateFromKin(L, K);
//  cout <<"INITIAL LOGIC STATE = " <<*L.start_state <<endl;

  LGP_Tree lgp(K, L);

//  lgp.verbose = 0;
//  rai::timerStart();
//  for(uint d=1;d<10;d++){
//    lgp.buildTree(d);
//    MNodeL all = lgp.root->getAll();
//    cout <<"d= " <<d <<" #= " <<all.N <<" t= " <<rai::timerRead(true) <<endl;
//  }

//  lgp.updateDisplay();
//  lgp.player();

//  lgp.optFixedSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)", BD_seq);
//  rai::wait(); return;

//  lgp.optFixedSequence("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (hit stickTip redBall) (grasp baxterL redBall) ", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (hitSlide stickTip redBall table1) (grasp baxterL redBall) ", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (hit stickTip redBall) (grasp baxterR redBall) ", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (throw baxterR stick) (hit stickTip redBall) (grasp baxterR redBall) ", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (throw baxterR stick) (hitSlide stickTip redBall table1) (grasp baxterR redBall) ", BD_path);
  lgp.optFixedSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1)", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (push stickTip redBall table1) (graspSlide baxterR redBall table1)", BD_path);
//  lgp.optFixedSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (grasp baxterR redBall)", BD_path);
//  lgp.optFixedSequence("(graspSlide baxterR stick table1) (hitSlide stickTip redBall table1) (grasp baxterL redBall) ", BD_path);

//  lgp.optMultiple({
//                    "(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stickTip redBall table1) (graspSlide baxterR redBall table1) ",
//                    "(grasp baxterR stick) (handover baxterR stick baxterL) (push stickTip redBall table1) (grasp baxterR redBall) ",
//                    "(grasp baxterR stick) (hitSlide stickTip redBall table1) (grasp baxterL redBall) ",
//                    "(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ",
//                    "(graspSlide baxterR stick table1) (hitSlide stickTip redBall table1) (grasp baxterL redBall) ",
//                    "(grasp baxterR stick) (hitSlide stickTip redBall table1) (graspSlide baxterL redBall table1) ",
//                  });

  rai::wait();  lgp.renderToVideo();

//  lgp.verbose=2;
//  lgp.getSymbolicSolutions(3);
//  lgp.run();


//  lgp.renderToVideo();
}

void solve1_seq_explicit(){
  rai::KinematicWorld K("../../models/RSSproblem-01.g");
  K.optimizeTree();
  K.makeObjectsFree({"stick"});

  KOMO komo;
  komo.setModel(K, false);

#if 0 //this is equivalent if 1 stepsPerPhase~
  komo.setDiscreteOpt(3);
#else
  komo.setTiming(3., 1, 1., 1);
  komo.setSquaredQuaternionNorms();
#endif
  komo.setSquaredQVelocities(0.,-1.,1e-1);
//  komo.setHoming(0., -1., 1e-2);

//  komo.add_StablePose({-1,0}, "stick");
  komo.addObjective(-1.,1., symbols2feature(FS_pose, {"stick"}, K), OT_eq, {}, 1e1, 1);

  komo.add_touch(1., 1., "baxterR", "stick");

//  komo.add_StableRelativePose({0,1}, "baxterR", "stick");
  komo.addObjective(1.,2., symbols2feature(FS_poseRel, {"baxterR", "stick"}, K), OT_eq, {}, 1e1, 1, +1);

  komo.add_touch(2., 2., "baxterL", "stick");

  //  komo.add_StableRelativePose({1,2}, "baxterR", "stick");
  komo.addObjective(2.,3., symbols2feature(FS_poseRel, {"baxterL", "stick"}, K), OT_eq, {}, 1e1, 1, +1);

  komo.optimize();

  while(komo.displayTrajectory()) {}
}


void solve5(){
  rai::KinematicWorld K("../../models/RSSproblem-05.g");
  FOL_World L(FILE("../../models/RSSfol.g"));
  initFolStateFromKin(L, K);

  LGP_Tree lgp(K, L);

//  lgp.player();

  lgp.optFixedSequence("(grasp baxterL blueBall) (grasp baxterR stick) (throw baxterL blueBall) (hit stick blueBall) (place3 blueBall bucket) ", BD_path);

//  lgp.optMultiple({
//                    "(grasp baxterL blueBall) (grasp baxterR stick) (throw baxterL blueBall) (hit stick blueBall) (place3 blueBall bucket) ",
//                    "(grasp baxterR stick) (handover baxterR stick baxterL) (hitSlide stick blueBall table1) (place3 blueBall bucket) ",
//                    "(grasp baxterR stick) (throw baxterR stick) (hitSlideAndSit stick blueBall table1) (place3 blueBall bucket) ",
//                    "(grasp baxterL blueBall) (throw baxterL blueBall) (place3 blueBall bucket) ",
//                    "(grasp baxterL stick) (hit stick blueBall) (place3 blueBall bucket) "
//                  });

  rai::wait();  lgp.renderToVideo();

//  lgp.run();
}

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);
//  rnd.clockSeed();

//  illustrate(); return 0;

  if(rai::checkParameter<uint>("problem")){
    //    buildTree(rai::getParameter<uint>("problem"));
    solve(rai::getParameter<uint>("problem"));
    return 0;
  }

//  solve1_seq_explicit();
  solve1();
//  solve5();

  return 0;
}
