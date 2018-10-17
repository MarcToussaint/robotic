#!/usr/bin/python3

import sys
sys.path.append('../ry')
from libry import *

K = Config()
D = K.view()

K.addFile('../test/lgp-example.g');

lgp = K.lgp("../test/fol.g");

lgp.optimizeFixedSequence("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");

input("Press Enter to continue...")
