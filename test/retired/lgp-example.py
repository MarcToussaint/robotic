#!/usr/bin/python3

import sys
sys.path.append('../build')
from libry import *

K = Config()
D = K.view()

K.addFile('../test/lgp-example.g');
K.makeObjectsConvex()

lgp = K.lgp("../test/fol.g");

lgp.walkToNode("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");
print(lgp.nodeInfo())

lgp.optBound(BT.path, True);

komo = lgp.getKOMOforBound(BT.path)
komo.display()

input("Press Enter to continue...")
