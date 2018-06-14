#!/usr/bin/python3

from ry import *

def main():
    K = Configuration()
    
    D = K.display()
    D.update(True)
    K.addFile('rai-robotModels/baxter/baxter.g');
    D.update(True)
    q = K.getJointState()
    print(q)

if __name__ == '__main__':
    main()
    input('quit?')
