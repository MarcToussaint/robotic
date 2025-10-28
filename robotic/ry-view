#!/usr/bin/env python3

import robotic as ry
import argparse
import os

parser = argparse.ArgumentParser(
    description='View a file (.g .yml .h5 or some mesh file). If config file, you can edit while viewing')

parser.add_argument('FILE', type=str,
                    help='g-file name')
parser.add_argument('-v', '--version', action='version',
                    version=f'%(prog)s -- robotic package version: {ry.__version__}, {ry.compiled()}')

def main():
    args = parser.parse_args()

    ext = os.path.splitext(args.FILE)[1]

    print('=== viewing file:', args.FILE)

    if ext in ['.g', '.yml']:
        print('    assuming this is a configuration')
        try:
            C = ry.Config()
            C.watchFile(args.FILE)
        except KeyboardInterrupt:
            exit(1)

    elif ext in ['.h5']:
        print('    assuming this is a h5 object (mesh & decomp & inertia)')
        C = ry.Config()
        C.addH5Object('object', args.FILE)
        print(C.write())
        C.view(True)

    else:
        print('    assuming this is a mesh file (using assimp to load)')
        C = ry.Config()
        f = C.addFrame('mesh')
        f.setMeshFile(args.FILE)
        print(C.write())
        C.view(True)

if __name__ == "__main__":
    main()
