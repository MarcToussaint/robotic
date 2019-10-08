#!/usr/bin/python3

import sys
sys.path.append('../rai/rai/ry')
from libry import *

K = Config()
D = K.view()

K.addFile('../rai-robotModels/pr2/pr2.g');
K.addFile('../test/kitchen.g');
print("joint names: ", K.getJointNames())
print("frame names: ", K.getFrameNames())

q = K.getJointState()
print('joint state: ', q)
q[2] = q[2] + 1.
K.setJointState(q)

X = K.getFrameState()
print('frame state: ', X)
X = X + .1
K.setFrameState(X.flatten().tolist())

q = K.getJointState()
print('joint state: ', q)
q[2] = q[2] + 1.
K.setJointState(q)

K.addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.3")
C = K.view(frame="camera")

input("Press Enter to continue...")
