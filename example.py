#!/usr/bin/python3

from ry import *

def main():
    K = Configuration()
    D = K.display()
    D.update(True)

    K.addFile('rai-robotModels/baxter/baxter.g');
    print("joint names: ", K.getJointNames())
    print("frame names: ", K.getFrameNames())
    D.update(True)
    
    q = K.getJointState()
    print('joint state: ', q)
    q[2] = q[2] + 1.
    K.setJointState(q)
    D.update(True)

    X = K.getFrameState()
    print('frame state: ', X)
    X = X + .1
    K.setFrameState(X)
    D.update(True)

    q = K.getJointState()
    print('joint state: ', q)
    q[2] = q[2] + 1.
    K.setJointState(q)
    D.update(True)

    K.addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(1. 1. 2)>" );
    D.update(True)

#    IK = K.getIK()
#    IK.optimize( [ ('eq', ['posDiff', 'baxterL', 'ball'], {}) ] )
#    D.update(True)
    
    

    
    

if __name__ == '__main__':
    main()
    input('quit?')
