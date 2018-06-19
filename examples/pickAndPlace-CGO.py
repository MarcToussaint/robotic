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

features = [
    ([-1,c0], ['eq', 'pose', obj1], {}),
    ([c1,c2], ['eq', 'pose', obj1], {}),
    ([c2,c3], ['eq', 'pose', obj1], {}),
    
    ([-1,c0], ['eq', 'pose', obj2], {}),
    ([c0,c1], ['eq', 'pose', obj2], {}),
    ([c1,c2], ['eq', 'pose', obj2], {}),
    
    #-- pick obj1
    ([c0], ['eq', 'dist', arm, obj1], {}),
    
    #-- place obj1
    ([c1], ['ineq', 'above', table, obj1], {}),
    ([c1], ['eq', 'aboveZ', table, obj1], {}),
    ([c1], ['sos', 'vec', obj1], {'v1': [0.,0.,1.], 'target':[0.,0.,1.]}),
    ([c0,c1], ['eq', 'poseDiff', arm, obj1], {}),
    
    #-- pick obj2
    ([c2], ['eq', 'dist', arm, obj2], {}),
    
    #-- place obj2
    ([c3], ['ineq', 'above', table, obj2], {}),
    ([c3], ['eq', 'aboveZ', table, obj2], {}),
    ([c3], ['sos', 'vec', obj2], {'v1': [0.,0.,1.], 'target':[0.,0.,1.]}),
    ([c2,c3], ['eq', 'poseDiff', arm, obj2], {}),
]

komo.optimize( features )

komo.getConfiguration(-1)
komo.getConfiguration(0)
komo.getConfiguration(1)
komo.getConfiguration(2)
komo.getConfiguration(3)
