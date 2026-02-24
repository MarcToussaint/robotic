#!/usr/bin/env python3

import argparse
import robotic as ry
import numpy as np

parser = argparse.ArgumentParser(
    description='Utility to test real robot basic operations')

parser.add_argument('-v', '--version', action='version',
                    version=f'%(prog)s -- robotic package version: {ry.version()}, {ry.compiled()}')

parser.add_argument('-two', help='use two arms', action="store_true")
parser.add_argument('-real', help='use the real robot', action="store_true")

parser.add_argument('-open', help='open gripper', action="store_true")
parser.add_argument('-close', help='close gripper', action="store_true")
parser.add_argument('-float', help='floating control (zero gains)', action="store_true")
parser.add_argument('-damp', help='damped control (zero position gains)', action="store_true")
parser.add_argument('-hold', help='hold positin', action="store_true")
parser.add_argument('-up', help='move up', action="store_true")
parser.add_argument('-home', help='move home', action="store_true")
parser.add_argument('-stretch', help='move stretch', action="store_true")

def main():
    args = parser.parse_args()
    np.set_printoptions(precision=4)
    
    C = ry.Config()

    if args.two:
        C.addFile(ry.raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"))
    else:
        C.addFile(ry.raiPath("../rai-robotModels/scenarios/pandaSingle.g"))

    bot = ry.BotOp(C, args.real)

    print('== status:')
    q = bot.get_q()
    l = C.getJointLimits()
    print('   q :', q)
    print('   lo:', q-l[0])
    print('   up:', l[1]-q)
    
    if args.close:
        bot.gripperMove(ry._left, 0, .05)
        while (not bot.gripperDone(ry._left)):
            bot.sync(C)

    if args.open:
        bot.gripperMove(ry._left)
        while (not bot.gripperDone(ry._left)):
            bot.sync(C)

    if args.float:
        bot.hold(floating=True, damping=False)
        bot.wait(C, forKeyPressed=True, forTimeToEnd=False)

    if args.damp:
        bot.hold(floating=True, damping=True)
        bot.wait(C, forKeyPressed=True, forTimeToEnd=False)

    if args.hold:
        bot.hold(floating=False, damping=False)
        bot.wait(C, forKeyPressed=True, forTimeToEnd=False)

    if args.up:
        q = bot.get_qHome()
        q[1] -= .5
        #if(q.N>7) q(8) -=.5;
        bot.moveTo(q, 1.)
        bot.wait(C, True, True)

    if args.home:
        q = bot.get_qHome()
        bot.moveTo(q, 1.)
        bot.wait(C, True, True)

    if args.stretch:
        q = np.array([0, -0.3, 0, -0.6,  0, 2., 0])
        bot.moveTo(q, 1.)
        bot.wait(C, True, True)

    print('== status:')
    q = bot.get_q()
    l = C.getJointLimits()
    print('   q :', q)
    print('   lo:', q-l[0])
    print('   up:', l[1]-q)
        
    del bot
    print('== used parameters:')
    ry.params_print()

if __name__ == "__main__":
    main()
