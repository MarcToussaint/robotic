#!/usr/bin/env python3

from robotic.src.yaml_helper import *
from robotic.src.urdf_io import *
from robotic.src.mesh_tool import *
import robotic as ry
import argparse
import glob
import os

parser = argparse.ArgumentParser(
    description='Utility to clean meshes in meshes/')

parser.add_argument('file', type=str, help='urdf file', nargs='?', default='none')

parser.add_argument('-view', help='view mesh', action="store_true", default=True)
parser.add_argument('-flipDaeYZ', help='view mesh', action="store_true", default=False)
parser.add_argument('-reverseRPY', help='reverse RPY convention', action="store_true", default=True)
parser.add_argument('-pruneRigidJoints', help='view mesh', action="store_true", default=True)
parser.add_argument('-recomputeInertias', help='view mesh', action="store_true")
parser.add_argument('-defaultMassDensity', help='view mesh', type=float, default=5.)
parser.add_argument('-minInertiaDiagonal', help='view mesh', type=float, default=1e-6)
parser.add_argument('-processMeshes', help='view mesh', action="store_true", default=True)
parser.add_argument('-meshlab', help='apply meshlab filters', action="store_true", default=False)

def convert(file, cfg):

    print('=== URDF CONVERT ===', file)

    # path, file = os.path.split(file)
    filebase, _ = os.path.splitext(file)

    if cfg.flipDaeYZ:
        ry.set_params({'assimp/daeFlipYZ': False})

    C = URDFLoader(file, visualsOnly=True, meshPathRemove='package://', reverseRPY=cfg.reverseRPY).C

    print('#frames raw: ', C.getFrameDimension())

    with open(f'{filebase}_raw_conv.yml', 'w') as fil:
        fil.write(C.asYaml())

    C.processStructure(cfg.pruneRigidJoints, True, False, False)
    if cfg.recomputeInertias:
        ry.set_params({"defaultMassDensity": cfg.defaultMassDensity})
        ry.set_params({"minInertiaDiagonal": cfg.minInertiaDiagonal})
    C.processInertias(cfg.recomputeInertias)
    C.processStructure(cfg.pruneRigidJoints, True, False, False)

    print('#frames processes: ', C.getFrameDimension())

    os.system('rm -Rf meshes/')
    C.writeMeshes('meshes/', copyTextures=True)

    with open(f'{filebase}_conv.yml', 'w') as fil:
        fil.write(C.asYaml())

    # yaml_write_dict(C.asDict(), f'{filebase}_conv.yaml')

    C.view(True)
    # C.animate()

    if cfg.processMeshes:
        for file in sorted(glob.glob('meshes/*.h5')):

            M = MeshTool(file)
            if M.tmesh is None:
                continue

            M.repair_meshlab(merge_threshold=1e-4)
            # M.repair_trimesh(mergeTolerance=1e-3)
            print('  watertight:', M.tmesh.is_watertight)
            print('  oriented:', M.tmesh.is_winding_consistent)

            M.report()
            # M.export_stl()
            M.export_h5(without_colors=True)

def main():
    args = parser.parse_args()

    if args.file=='none':
        args.file = '/home/mtoussai/git/rai-robotModels/allegro/allegro.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/z1/z1.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/panda/panda_arm_hand.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/g1/g1_description/g1_29dof.urdf'
        # args.file = '/home/mtoussai/git/rai-robotModels/ranger/ranger_mini.urdf'

    convert(args.file, args)

if __name__ == "__main__":
    main()
