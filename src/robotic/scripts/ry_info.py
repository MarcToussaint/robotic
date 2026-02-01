#!/usr/bin/env python3

import robotic as ry

def main():
    print()
    print('== Robotic Control Interface & Manipulation Planning Library')
    print('   https://pypi.org/project/robotic/')
    print('   pip package version:', ry.__version__)
    print('  ', ry.compiled())
    print('   raiPath:', ry.raiPath(''))
    print('   parameters set in rai.cfg:')
    ry.params_print()
    print('\n')

if __name__ == "__main__":
    main()
