#!/usr/bin/python3

from ry import *

def main():
    K = Configuration()
    D = K.display()
    D.update("empty configuration\n -- hit ENTER here to continue", True)

    K.addFile('rai-robotModels/pr2/pr2.g');
    K.addFile('test/kitchen.g');
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

    K.addFrame("camera", "head_tilt_link", "Q:<d(-90 1 0 0) d(180 0 0 1)> focalLength:.3")
    C = K.camera(frame="camera")
  
    K.addFrame("ball", "", "shape:sphere size:[0 0 0 .1] color:[1 1 0] X:<t(.8 .8 1.5)>" );
    D.update(True)
    
    K.addFrame("hand", "pr2L", "shape:ssBox size:[.3 .2 .1 .01] color:[1 1 0] Q:<t(0 0 0)>" );
    D.update(True);

    dist = K.getPairDistance("hand", "ball");
    print('distance: ', dist);
    D.update(True);

    komo = K.komo()
    komo.optimize( [ ([], 'eq', ['posDiff', 'pr2L', 'ball'], {}) ] )

    C.update()
    D.update(True)


    
    

    
    

if __name__ == '__main__':
    main()
    input('quit?')
