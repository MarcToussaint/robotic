#!/usr/bin/python3

import sys
sys.path.append('../ry')
from libry import *

K = Config()
D = K.view()
K.addFile('../test/kitchen.g')
K.addFile('../rai-robotModels/pr2/pr2.g')

K.addFrame('item1', 'sink1', 'type:ssBox joint:rigid Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact' )
K.addFrame('item2', 'sink1', 'type:ssBox joint:rigid Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact' )
K.addFrame('tray', 'stove1', 'type:ssBox joint:rigid Q:<t(.0 .0 .42)> size:[.2 .2 .05 .02] color:[0. 1. 0.], contact' )

obj1 = "item2";
obj2 = "item1";
tray = "tray";
arm = "pr2L";
table = "_12";

komo = K.komo_CGO(6)

komo.activateCollisionPairs([(obj1, obj2)]);
komo.addObjective([], OT.eq, FS.accumulatedCollisions);
komo.addObjective([], OT.ineq, FS.jointLimits);

komo.add_StableRelativePose([0, 1], arm, obj1);
komo.add_StableRelativePose([2, 3], arm, obj2);
komo.add_StableRelativePose([4, 5], arm, tray);

komo.add_StableRelativePose([1,2,3,4,5], tray, obj1);
komo.add_StableRelativePose([3,4,5], tray, obj2);

komo.add_StablePose([-1,0], obj1);
komo.add_StablePose([-1,0,1,2], obj2);
komo.add_StablePose([-1,0,1,2,3,4], tray);

komo.add_grasp(0, arm, obj1);
komo.add_place(1, obj1, tray);

komo.add_grasp(2, arm, obj2);
komo.add_place(3, obj2, tray);

komo.add_grasp(4, arm, tray);
komo.add_place(5, tray, table);

komo.optimize()

K.setFrameState( komo.getConfiguration(4) )
K.getJointState()

komo.displayTrajectory()

