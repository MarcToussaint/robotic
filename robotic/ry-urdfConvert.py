#!/usr/bin/env python3

# from config_mujoco import *
# from config_urdf import *
# from mesh_helper import *
from robotic.src.yaml_helper import *
import robotic as ry
import argparse
import glob
import os

parser = argparse.ArgumentParser(
    description='Utility to clean meshes in meshes/')

parser.add_argument('file', type=str, help='urdf file', nargs='?', default='none')

parser.add_argument('-view', help='view mesh', action="store_true", default=True)
parser.add_argument('-flipDaeYZ', help='view mesh', action="store_true", default=False)
parser.add_argument('-pruneRigidJoints', help='view mesh', action="store_true")
parser.add_argument('-recomputeInertias', help='view mesh', action="store_true")
parser.add_argument('-processMeshes', help='view mesh', action="store_true", default=True)
parser.add_argument('-meshlab', help='apply meshlab filters', action="store_true", default=False)

def main():
    args = parser.parse_args()

    if args.file=='none':
        # args.file = '/home/mtoussai/git/rai-robotModels/pr2/pr2.urdf'
        args.file = '/home/mtoussai/git/rai-robotModels/panda/panda_arm_hand.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/g1/g1_description/g1_29dof.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/ranger/ranger_mini.urdf'

    print('=== URDF CONVERT ===', args.file)

    path, file = os.path.split(args.file)
    filebase, _ = os.path.splitext(file)

    if args.flipDaeYZ:
        ry.params_add({'assimp/daeFlipYZ': False})

    C = ry.URDFLoader(args.file, visualsOnly=True, meshPathRemove='package://').C

    C.processStructure(args.pruneRigidJoints, True, False, False)
    C.processInertias(args.recomputeInertias)
    C.processStructure(args.pruneRigidJoints, True, False, False)

    os.system('rm -Rf meshes/')
    C.writeMeshes('meshes/', copyTextures=True)

    print('#frames: ', C.getFrameDimension())
    with open(f'{filebase}_conv.g', 'w') as fil:
        #yaml.dump(C.asDict(), file, default_flow_style=False)
        fil.write(C.write())

    yaml_write_dict(C.asDict(), f'{filebase}_conv.yml')

    C.view(True)
    # C.animate()

    if args.processMeshes:
        for file in sorted(glob.glob('meshes/*.h5')):

            M = ry.MeshHelper(file)
            if M.mesh is None:
                continue

            M.repair(mergeTolerance=1e-4)
            print('  watertight:', M.mesh.is_watertight)
            print('  oriented:', M.mesh.is_winding_consistent)
            M.report()
            M.export_h5()

if __name__ == "__main__":
    main()
