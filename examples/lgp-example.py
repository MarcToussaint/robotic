#!/usr/bin/python3

from ry import *

K = Configuration()
D = K.camera()

K.addFile('../test/lgp-example.g');
D.update()


lgp = K.lgp("../test/fol.g");

lgp.optimizeFixedSequence("(grasp baxterR stick) (push stickTip redBall table1) (grasp baxterL redBall) ");

D.update(True)

