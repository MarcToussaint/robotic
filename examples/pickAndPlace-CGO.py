#!/usr/bin/python3

from ry import *

K = Configuration()
D = K.camera()
K.addFile('../test/kitchen.g')
K.addFile('../rai-robotModels/pr2/pr2.g')

K.addFrame('item1', 'sink1', 'type:ssBox Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact' )
K.addFrame('item2', 'sink1', 'type:ssBox Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact' )
K.addFrame("tray", "stove1", "type:ssBox Q:<t(.0 .0 .42)> size:[.2 .2 .05 .02] color:[0. 1. 0.], contact" )

obj1 = 'item2'
obj2 = 'item1'
arm='pr2L'
tray = "tray";
table='_12'

T = 6
komo = K.komo_CGO(T)
#komo.makeObjectsFree([obj1, obj2])

komo.addObjective(confs=[3], type='ineq', feature='dist', frames=[obj1, obj2], target=[-.05]); # distance between objects!
komo.addObjective(type='eq', feature='coll');
komo.addObjective(type='ineq', feature='limits');

komo.add_StableRelativePose(confs=[0, 1], gripper=arm, object=obj1);
komo.add_StableRelativePose(confs=[2, 3], gripper=arm, object=obj2);
komo.add_StableRelativePose(confs=[4, 5], gripper=arm, object=tray);

komo.add_StableRelativePose(confs=[1,2,3,4,5], gripper=tray, object=obj1);
komo.add_StableRelativePose(confs=[3,4,5], gripper=tray, object=obj2);

komo.add_StablePose(confs=[-1, 0], object=obj1);
komo.add_StablePose(confs=[-1, 0, 1, 2], object=obj2);
komo.add_StablePose(confs=[-1, 0, 1, 2, 3, 4], object=tray);

komo.add_grasp(0, arm, obj1)
komo.add_place(1, obj1, tray)

komo.add_grasp(2, arm, obj2)
komo.add_place(3, obj2, tray)

komo.add_grasp(4, arm, tray);
komo.add_place(5, tray, table);

# komo.add_resting(-1, 0, obj1)
# komo.add_restingRelative(0, 1 , obj1, arm)
# komo.add_resting(1, 2, obj1)
# komo.add_resting(2, 3, obj1)

# komo.add_resting(-1, 0, obj2)
# komo.add_resting(0, 1, obj2)
# komo.add_resting(1, 2, obj2)
# komo.add_restingRelative(2, 3 , obj2, arm)

komo.optimize()

for t in range(-1, T):
    komo.getConfiguration(t);
    D.update(True)
