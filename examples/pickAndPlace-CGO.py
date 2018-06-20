#!/usr/bin/python3

from ry import *

K = Configuration()
D = K.display()
K.addFile('../test/kitchen.g')
K.addFile('../rai-robotModels/pr2/pr2.g')

K.addFrame('item1', 'sink1', 'type:ssBox Q:<t(-.1 -.1 .52)> size:[.1 .1 .25 .02] color:[1. 0. 0.], contact' )
K.addFrame('item2', 'sink1', 'type:ssBox Q:<t(.1 .1 .52)> size:[.1 .1 .25 .02] color:[1. 1. 0.], contact' )
    
obj1 = 'item2'
obj2 = 'item1'
arm='pr2L'
table='_13'
c0=0; c1=1; c2=2; c3=3;

komo = K.komo_CGO(4)
komo.makeObjectsFree([obj1, obj2])

komo.add_grasp(c0, arm, obj1)
komo.add_place(c1, obj1, table)
komo.add_grasp(c2, arm, obj2)
komo.add_place(c3, obj2, table)

komo.add_resting(-1, c0, obj1)
komo.add_restingRelative(c0, c1 , obj1, arm)
komo.add_resting(c1, c2, obj1)
komo.add_resting(c2, c3, obj1)

komo.add_resting(-1, c0, obj2)
komo.add_resting(c0, c1, obj2)
komo.add_resting(c1, c2, obj2)
komo.add_restingRelative(c2, c3 , obj2, arm)

komo.optimize()

komo.getConfiguration(-1)
komo.getConfiguration(0)
komo.getConfiguration(1)
komo.getConfiguration(2)
komo.getConfiguration(3)
