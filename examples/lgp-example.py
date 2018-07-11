#!/usr/bin/python3

from ry import *

K = Configuration()
D = K.camera()

K.addFile('lgp-example.g');
D.update(True)


komo = K.komo()
komo.addObjective(type='eq', feature='posDiff', frames=['ball', 'hand'])
komo.optimize()

komo.clearObjectives()
komo.addObjective(type='eq', feature='posDiff', frames=['hand', 'ball'], target=[.1, .1, .1])
komo.optimize()


