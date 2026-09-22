#!/usr/bin/env python3

import argparse
import robotic as ry
import numpy as np

parser = argparse.ArgumentParser(
    description='Utility to test real robot basic operations')

parser.add_argument('robot', type=str, help='name of robot (e.g. franka.16 or trossen.3)', nargs='?', default='franka.16')

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

    if args.robot=="franka.16":
        C.addFile("$RAI_PATH/scenarios/pandaSingle.g")
    elif args.robot=="franka.17":
        C.addFile("$RAI_PATH/scenarios/pandaSingle.g")
    elif args.robot=="franka.16.17":
        C.addFile("$RAI_PATH/scenarios/pandasTable.g")
    elif args.robot=="trossen.5":
        C.addFile("$RAI_PATH/trossen/trossen.yml")

    bot = ry.BotOp(C, useRealRobot=args.real, auto_launch=False)

    if args.robot=="franka.16":
        bot.launch_franka("172.16.0.2", C.getFrame("l_panda_base"), also_gripper=True)
    elif args.robot=="franka.17":
        bot.launch_franka("172.17.0.2", C.getFrame("l_panda_base"), also_gripper=True)
    elif args.robot=="franka.16.17":
        bot.launch_franka("172.16.0.2", C.getFrame("l_panda_base"), also_gripper=True)
        bot.launch_franka("172.17.0.2", C.getFrame("r_panda_base"), also_gripper=True)
    elif args.robot=="trossen.5":
        bot.launch_trossen("192.168.1.5")
    else:
        raise Exception("robot '{args.robot}' not captured")

    print('== status:')
    q = bot.get_q()
    l = C.getJointLimits()
    print('   q :', q)
    print('   lo:', q-l[0])
    print('   up:', l[1]-q)
    
    if args.close:
        bot.gripperMove(0, 0, .05)
        while (not bot.gripperDone(0)):
            bot.sync(C)

    if args.open:
        bot.gripperMove(0)
        while (not bot.gripperDone(0)):
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
    print('')

if __name__ == "__main__":
    main()
