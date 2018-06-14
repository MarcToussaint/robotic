#!/usr/bin/python3

from ry import *

K = Kin()

def bla():
    D = K.display()
    D.update(True)
    K.addFile('rai-robotModels/kuka_drake/kuka.g');
    D.update(True)
    q = K.getJointState()
    print(q)

bla()

input('quit')
