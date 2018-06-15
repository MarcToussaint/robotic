#!/usr/bin/python3

from ry import *

def main():
    K = Configuration()
    D = K.display()
    D.update("empty configuration\n -- hit ENTER here to continue", True)

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

    K.addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(.8 .8 1.5)>" );
    D.update(True)

    komo = K.komo()
    komo.optimize( [ ('eq', ['posDiff', 'baxterL', 'ball'], {}) ] )
    D.update(True)
    
    

    
    

if __name__ == '__main__':
    main()
    input('quit?')
