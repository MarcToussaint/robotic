#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Core/graph.h>
#include <Gui/opengl.h>
#include <Kin/proxy.h>
#include <Kin/frame.h>
#include <Kin/kin_swift.h>
#include <Kin/contact.h>
#include <Kin/TM_FlagConstraints.h>
#include <Kin/kin_physx.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_NewtonEuler.h>
#include <Kin/TM_energy.h>
#include <Kin/TM_PairCollision.h>
#include <Kin/switch.h>
#include <Kin/TM_time.h>
#include <Kin/TM_angVel.h>

//===========================================================================

void testQuat(){

#if 1
//  rai::Quaternion
  VectorFunction f = [](arr& y, arr& J, const arr& x){
    CHECK_EQ(x.N, 8,"");
    arr an, bn, Jan, Jbn;
    quat_normalize(an, Jan,  x({0,3}));
    quat_normalize(bn, Jbn,  x({4,7}));
    arr Ja, Jb;
    //    quat_concat(y, Ja, Jb, an, bn);
    quat_diffVector(y, Ja, Jb, an, bn);

    if(&J) J = catCol(Ja * Jan, Jb * Jbn);

//    quat_normalize(y, J, a);
  };

  arr x(8);
  for(uint k=0;k<20;k++){
    x = 1.;
    rndGauss(x, 1.);
    checkJacobian(f, x, 1e-6);
  }

  return;
#endif

  rai::Quaternion q1 = 0;
  rai::Quaternion q2 = 0;

  q1.setRadX(RAI_PI/2.);
  q2.setRadY(RAI_PI/2.);
  q1.setRandom();
  q2.setRandom();

  cout <<q1 <<' ' <<q2 <<endl;

  rai::Quaternion d = q2/q1;


//  d.flipSign();

  cout <<d <<endl;

  arr w = (q1 / d.getVec()).getArr();

  cout <<w <<endl;

  cout <<(-q1 * q2).getVec() <<endl;

  rai::Quaternion W;
  W.setVec(w);

  cout <<q1*W <<endl;

  cout <<"error = " <<q2.sqrDiff(q1*W) <<endl;

}

//===========================================================================

void angularVel(){
  rai::KinematicWorld K;
  K.addFrame("world");
  K.addObject("box", rai::ST_ssBox, {.2, .3, .4, .02}, {}, -.1, NULL, {.0, .0, .5});
  K.watch(true);

  KOMO komo(K);
  komo.setTiming(1., 20, 1., 2);
  komo.setSquaredQuaternionNorms();

  komo.addSwitch_magic(-1., -1., "world", "box", -1.);
  komo.addObjective({}, OT_sos, FS_position, {"box"}, {}, {}, 1);
  komo.addObjective({}, OT_sos, FS_angularVel, {"box"}, {}, {0., 0., 1.});

  komo.reset();
  komo.reportProblem();
  komo.animateOptimization=true;
  komo.run();
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: " <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;
  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true));
}

//===========================================================================

void passive_elasticBounce(){
  rai::KinematicWorld K;
  K.addFrame("base");
  K.addObject("floor", rai::ST_ssBox, {1., 1., .1, .02}, {}, -1., NULL, {0.,-.6,.5}, {1., .1, 0., 0.});
  K.addObject("ball",  rai::ST_ssBox, {.0, .4, .1, .05}, {}, -.1, NULL, {.0, .0, 1.05});
//  (new rai::Inertia(*K["ball"])) -> defaultInertiaByShape();

  KOMO komo(K);
  komo.setTiming(1., 20, 1., 2);
  komo.setSquaredQuaternionNorms();
#if 0
  komo.setTimeOptimization();
  komo.addSwitch_dynamic(.2, -1., K.frames.first()->name, "ball");

  komo.addContact_elasticBounce( .4, "floor", "ball", .5, .5);
#else
  komo.addSwitch_magicTrans(-1., .1, K.frames.first()->name, "ball", 1e-5);
  komo.addSwitch_dynamic(.1, -1., K.frames.first()->name, "ball");
  komo.addContact_elasticBounce( .5, "floor", "ball", .5, .5);
#endif

  komo.optimize();
  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true, true, "z.vid/"));
}

//===========================================================================

void passive_elasticBounce2(){
  rai::KinematicWorld K;
  K.addFrame("base");
  K.addObject("floor", rai::ST_ssBox, {1., 1., .1, .02}, {}, -1., NULL, {0., .0,.5});
  K.addObject("ball",  rai::ST_sphere, {}, {}, .05, NULL, {.0, .0, 1.});
//  (new rai::Inertia(*K["ball"])) -> defaultInertiaByShape();

  KOMO komo;
  komo.setModel(K, false);
  komo.setPathOpt(4.5, 10., .2);
  komo.setTimeOptimization();

  komo.addSwitch_dynamic(.1, -1., K.frames.first()->name, "ball");
  komo.addContact_elasticBounce( 1., "floor", "ball", .8);
  komo.addContact_elasticBounce( 2., "floor", "ball", .8);
  komo.addContact_elasticBounce( 3., "floor", "ball", .8);
  komo.addContact_elasticBounce( 4., "floor", "ball", .8);

  komo.optimize();
  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true, false, "z.vid/"));
}

//===========================================================================

void passive_slidePermanent(){
  rai::KinematicWorld K;
  K.addObject("floor", rai::ST_ssBox, {1., 1., .1, .02}, {}, -1., NULL, {0.,.5,.5}, {1., .1, 0., 0.});
  K.addObject("ball",  rai::ST_ssBox, {.4, .4, .2, .05}, {}, -.1, NULL, {.0, .0, .62}, {1., .1, 0., .1});

  KOMO komo(K);
  komo.setTiming(1., 40, 1., 2);
  komo.setSquaredQuaternionNorms();
//  komo.setTimeOptimization();

  komo.addSwitch_dynamic(0, -1., K.frames.first()->name, "ball");
//  komo.addContact_noFriction(.1, -1., "floor", "ball");
  komo.addContact_Complementary(0., -1., "floor", "ball");

  komo.verbose=2;
  komo.optimize();
  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true, false, "z.vid/"));
}

//===========================================================================

void passive_stickyPermanent(){
  rai::KinematicWorld K;
  K.addObject("floor", rai::ST_ssBox, {1., 1., .1, .02}, {}, -1., NULL, {0.,0., .5}, {1., .3, 0., 0.});
  K.addObject("ball",  rai::ST_ssBox, {.4, .2, .4, .05}, {}, -.1, NULL, {.0, .0, 1.1}, {1., 0., 0., .3});

  KOMO komo(K);
  komo.setTiming(1., 20, 1., 2);
  komo.setSquaredQuaternionNorms();
  komo.setTimeOptimization();

  komo.addSwitch_dynamic(0, -1., K.frames.first()->name, "ball");
  komo.addContact_stick( .21, -1., "floor", "ball");

  komo.verbose=2;
  komo.optimize();
  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true, false, "z.vid/"));
}

//===========================================================================

void plan(){
  rai::KinematicWorld K("gravity.g");
//  K.optimizeTree(false);

  KOMO komo;
  komo.setModel(K, true);
  komo.setTiming(1., 10, 1., 2);
  komo.setSquaredQuaternionNorms();
//  komo.setTimeOptimization();

//  komo.addSwitch_dynamic(.2, -1., K.frames.first()->name, "block");
  komo.addObjective(.0, -1., new TM_NewtonEuler(komo.world, "block"), OT_eq, NoArr, 1e-1, 2);
//  komo.addObjective(.0, -1., new TM_AngVel(komo.world, "block"), OT_eq, NoArr, 1e-1, 1);
//  komo.addObjective(.1, -1., new TM_NewtonEuler(komo.world, "block"), OT_eq, NoArr, 1e-1, k_order, +1, -1);

  //  Task *te = komo.setTask(-1, -1., new TM_Energy(), OT_ineq, NoArr, 1e-1, 2);

//  komo.add_collision(false);

//  komo.addObjective(-1., -1., new TM_ContactConstraints(), OT_eq, NoArr, 1e1);

//  komo.addContact_Complementary(-1., -1., "table1", "block" );
  komo.addContact_elasticBounce(.4, "table1", "block", .4 );

  komo.verbose=2;
  komo.reset();
//  komo.checkGradients(); return;
  komo.reportProblem();
  komo.animateOptimization=true;
  komo.run();
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: " <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;
  komo.checkGradients();// return;

//  komo.checkGradients();

  while(komo.displayTrajectory(-.1, true));
}

//===========================================================================

/* TODO:
 * DONE refactor: add a kinematic switch that adds and deletes contacts;
 * DONE add a komo.setKontact(startTime, stopTime) to create two kinematic switches
 * NO! change time optim to deltas
 * DONE within Contact: remove a_rel, etc; just have a CollisionPair pointe
 * DONE delete this pointer whenever 'calc F from q'
 * energy constraint -> locally of all involved collision objects
 */

void jumpingBall(){
  rai::KinematicWorld K("jumps.g");
//  K.optimizeTree(false);

  auto *jt = new rai::Joint(*K["world"]); jt->type = rai::JT_time; jt->H = 0.;

  KOMO komo;
  komo.setModel(K, true);
  komo.deactivateCollisions("boxBo", "boxLe");
  komo.deactivateCollisions("boxBo", "boxBa");
  komo.deactivateCollisions("boxLe", "boxBa");
  komo.setTiming(1., 20, 4., 2);
//  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities();
  komo.setSquaredQuaternionNorms();

  //-- set a time optim objective
  komo.addObjective(0., -1., new TM_Time(), OT_sos, {}, 1e1, 1); //smooth time evolution
  komo.addObjective(0., -1., new TM_Time(), OT_sos, {komo.tau}, 1e-1, 0); //prior on timing

  Objective *t = komo.addObjective(0., -1., new TM_NewtonEuler(komo.world, "ball"), OT_sos, NoArr, 1e-1, 2);
  t->vars(0) = 0.;
  t->vars(1) = 0.;

//  Task *te = komo.setTask(-1., -1., new TM_Energy(), OT_ineq, NoArr, 1e1, 2);
//  te->vars(0) = 0.;
//  te->vars(1) = 0.;

  komo.add_collision(false);

//  komo.addObjective(0., -1., new TM_ContactConstraints(), OT_sos, NoArr, 3e1);
  komo.add_touch(1., 1., "target", "ball", OT_sos, NoArr, 3e1);

  komo.addContact_slide(.6, .6, "boxBo", "ball" );
  komo.addContact_slide(.8, .8, "boxLe", "ball" );

  komo.verbose=2;
  komo.reset();
  komo.reportProblem();
  komo.animateOptimization=true;
  komo.run();
//  komo.animateOptimization=false;
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;

//  komo.checkGradients();

  while(komo.displayTrajectory(1., true));

  komo.displayTrajectory(1., false, true, "z.vid/");
}

//===========================================================================

void rollingBall(){
  rai::KinematicWorld K("roll.g");
//  K.optimizeTree(false);

  KOMO komo;
  komo.setModel(K, true);
  komo.setTiming(1., 50, 4., 2);
  komo.setSquaredQuaternionNorms();
  komo.setTimeOptimization();


  Objective *t = komo.addObjective(0., -1., new TM_NewtonEuler(komo.world, "ball"), OT_sos, NoArr, 1e-1, 2);
  t->vars(0) = 0.;
  t->vars(1) = 0.;

  Objective *te = komo.addObjective(.4, .7, new TM_Energy(), OT_ineq, NoArr, 1e1, 2);
//  te->vars(0) = 0.;
//  te->vars(1) = 0.;

  komo.add_collision(false);

  komo.add_touch(1., 1., "target", "ball", OT_sos, NoArr, 3e1);

  komo.addContact_noFriction(.2, .7, "wall2", "ball" );
  komo.addContact_noFriction(.8, 1., "wall1", "ball" );

  komo.verbose=2;
  komo.reset();
  komo.reportProblem();
  komo.animateOptimization=true;
  komo.run();
//  komo.animateOptimization=false;
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;

//  komo.checkGradients();

  while(komo.displayTrajectory(1., true));

  komo.displayTrajectory(1., false, true, "z.vid/");
}

//===========================================================================

void hittingBall(){
  rai::KinematicWorld K("hit.g");
//  K.optimizeTree(false);

  KOMO komo;
  komo.setModel(K, true);
  komo.setTiming(1., 20, 4., 2);
  komo.setSquaredQuaternionNorms();
  komo.setTimeOptimization();

  Objective *t = komo.addObjective(0., -1., new TM_NewtonEuler(komo.world, "ball", true), OT_eq, NoArr, 1e-1, 2);
  t->vars(0) = 0.;
  t = komo.addObjective(0., -1., new TM_NewtonEuler(komo.world, "ball2", true), OT_eq, NoArr, 1e-1, 2);
  t->vars(0) = 0.;

  komo.add_collision(false);

  komo.add_touch(1., 1., "target", "ball", OT_sos, NoArr, 3e1);
  komo.addObjective(0., .1, new TM_Default(TMT_pos, komo.world, "ball"), OT_eq, {}, 1e1, 1);

  komo.addContact_noFriction(.05, -1., "wall1", "ball" );
  komo.addContact_noFriction(.2, -1., "wall1", "ball2" );
  komo.addContact_noFriction(.4, .4, "ball2", "ball" );
//  komo.addContact_elasticBounce(.4, "ball2", "ball", .8, 0. );

  komo.verbose=2;
  komo.reset();
  komo.reportProblem();
  komo.animateOptimization=true;
  komo.run();
//  komo.animateOptimization=false;
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;

//  komo.checkGradients();

  while(komo.displayTrajectory(-1., true));

//  komo.displayTrajectory(-.1, false, "z.vid/");
}

//===========================================================================

void boxProblem(){
  rai::KinematicWorld K("boxProblem.g");
//  K.optimizeTree(false);

  KOMO komo;
  komo.setModel(K, true);
  komo.setTiming(4., 10, .2, 2);
  komo.setSquaredQuaternionNorms();
  komo.setTimeOptimization();

  rai::String obj = "ballR"; //"block"; //

  //a bit awkward: starting with .05 is what we need to exclude the starting state
  komo.addSwitch_magic(.0, .5, K.frames.first()->name, obj, 0.);
//  komo.addSwitch_dynamic(.1, -1., K.frames.first()->name, obj);
  komo.addSwitch_dynamicTrans(.7, -1., K.frames.first()->name, obj);

//  komo.addObjective({0., .6}, OT_ineq, FS_position, {obj}, arr(1,3,{0.,1.,0.}), {.8});


  komo.addObjective({}, OT_sos, FS_accumulatedCollisions, {});

//  komo.add_touch(1., 1., "target", obj, OT_eq, NoArr, 3e1);
  komo.addObjective({4.}, OT_eq, FS_distance, {"target", obj}, {3e1});

  komo.addContact_elasticBounce(1., "boxBo", obj, .9);
  komo.addContact_elasticBounce(2., "boxBo", obj, .9);
  komo.addContact_elasticBounce(3., "boxBo", obj, .9);

  komo.animateOptimization=true;
  komo.verbose=2;
  komo.optimize();
//  cout <<komo.getContacts() <<endl;

  arr taus = komo.getPath_tau();
  FILE("z.tau") <<~~taus;
//  gnuplot("plot 'z.dat' us 0:1");
  cout <<"TIMES: total=" <<sum(taus) <<taus <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;
  komo.checkGradients();

//  while(komo.displayTrajectory(-1., true));

//  komo.displayTrajectory(1., false, true, "z.vid/");

  for(;;){
    rai::wait();
    komo.optimize(true);
  }
}

//===========================================================================

void boxProblemSkeleton(){
  rai::KinematicWorld K("boxProblem.g");

  KOMO komo;
  komo.setModel(K, false);
  komo.setPathOpt(4., 10., .2);
  komo.setTimeOptimization();

  rai::String obj = "ball"; //"block"; //

//  komo.addObjective({}, OT_sos, FS_accumulatedCollisions, {});

  Skeleton S = {
    { 0., .5, SY_magic, {obj} },
    { .7, 4., SY_dynamicTrans, {obj} },
    { 1., 1., SY_bounce, {"boxBo", obj} },
    { 2., 2., SY_bounce, {"boxLe", obj} },
    { 3., 3., SY_bounce, {"boxBo", obj} },
    { 4., 4., SY_touch, {"target", obj} }
  };
  komo.setSkeleton(S);

  komo.animateOptimization=true;
  komo.verbose=2;
  komo.optimize();
//  cout <<komo.getContacts() <<endl;

  arr taus = komo.getPath_tau();
  FILE("z.tau") <<~~taus;
//  gnuplot("plot 'z.dat' us 0:1");
  cout <<"TIMES: total=" <<sum(taus) <<taus <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;
  komo.checkGradients();

  while(komo.displayTrajectory(-1., true, true, "z.vid/"));

//  komo.displayTrajectory(1., false, true, "z.vid/");

//  for(;;){
//    rai::wait();
//    komo.optimize(true);
//  }
}

//===========================================================================

void blocks(){
  rai::KinematicWorld K("boxProblem.g");
//  K.optimizeTree(false);

  KOMO komo;
  komo.setModel(K, true);
  komo.setTiming(1., 40, 1., 2);
  komo.deactivateCollisions("boxBo", "boxLe");
  komo.deactivateCollisions("boxBo", "boxBa");
  komo.deactivateCollisions("boxLe", "boxBa");
  komo.activateCollisions("boxBo", "block");
  komo.setSquaredQuaternionNorms(0., -1., 1e0);
//  komo.setSquaredQAccelerations(0., -1., 1e-2);

//  komo.setTimeOptimization();

  komo.addSwitch_magic(.0, .1, K.frames.first()->name, "block", 0.);
  komo.addSwitch_dynamic(.1, -1., K.frames.first()->name, "block");

//  komo.addObjective(0.4, -1., OT_ineq, FS_energy, {}, {1e-0} ); //new TM_Energy(), OT_ineq, NoArr, 1e-1, 2);

  komo.add_collision(false, 0., 1e2);

//  komo.addObjective(0., -1., OT_eq, FS_contactConstraints, {}, {3e1} ); // new TM_ContactConstraints(), OT_eq, NoArr, 3e1);
  komo.add_touch(1., 1., "target", "block", OT_eq, NoArr, 3e1);

//  komo.addContact(.4, .4, "boxBo", "block" );
  komo.addContact_slide(.6, .6, "boxBo", "block" );
//  komo.addContact(.8, .8, "boxBo", "block" );

//  komo.addObjective(.0, .55, new TM_AngVel(komo.world, "block"), OT_eq, {0., .0, 0.});
//  komo.addObjective(.6, -1., new TM_AngVel(komo.world, "block"), OT_eq, {0.01, .01, 0.});

  komo.verbose=2;
  komo.reset();
  komo.reportProblem();
//  komo.checkGradients(); return;
  komo.animateOptimization=true;
  komo.run();
  komo.animateOptimization=false;
  komo.checkGradients();// return;
  komo.getReport(true);
  cout <<komo.getContacts() <<endl;
  cout <<"TIMES: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
  cout <<"ENERGIES: " <<komo.getPath_energies() <<endl;

//  komo.checkGradients();

  while(komo.displayTrajectory(-1., true));

  komo.displayTrajectory(1., false, true, "z.vid/");
}

//===========================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  int mode = rai::getParameter<int>("mode", 1);
//  testQuat();
  switch(mode){
    case 0: angularVel(); break;
    case 7: passive_elasticBounce(); break;
    case 8: passive_elasticBounce2(); break;
    case 9: passive_slidePermanent(); break;
    case 10: passive_stickyPermanent(); break;
//  case 0: testQuat(); break;
  case 1: plan(); break;
  case 2: jumpingBall(); break;
  case 3: rollingBall(); break;
  case 4: hittingBall(); break;
//  case 5: boxProblem(); break;
    case 5: boxProblemSkeleton(); break;
  case 6: blocks(); break;
  }

  return 0;
}
