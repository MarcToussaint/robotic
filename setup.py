from setuptools import setup, dist
#from setuptools.command.install import install
from setuptools.command.build_ext import build_ext
import setuptools
import os
import glob
import subprocess
import platform

class BinaryDistribution(dist.Distribution):
    def has_ext_modules(foo):
        return True

class CustomCommand(build_ext):
    """Customized setuptools build_ext command"""
    def run(self):
        version = platform.python_version_tuple()[0] + platform.python_version_tuple()[1]
        print('[rai] custom copy lib, python version tag:', version)
        subprocess.check_call('cp -f build/libry*'+version+'*.so robotic/libry.so', shell=True)
        subprocess.check_call('strip --strip-unneeded robotic/libry.so', shell=True)
        build_ext.run(self)


setup(
    name='robotic',
    packages=['robotic'],
    package_data={'robotic': ['libry.so', 'rai-robotModels/*/*', 'rai-robotModels/*/*/*', 'rai-robotModels/*/*/*/*']},
    include_package_data=True,
    cmdclass={
        'build_ext': CustomCommand,
    },

    # data_files=[
    #     ('rai-robotModels/objects/',  glob.glob('rai-robotModels/objects/*.g')),
    #     ('rai-robotModels/panda/',  glob.glob('rai-robotModels/panda/*.g')),
    #     ('rai-robotModels/panda/meshes/', glob.glob('rai-robotModels/panda/meshes/*.ply')),
    #     ('rai-robotModels/pr2/', glob.glob('rai-robotModels/pr2/*.g')),
    #     ('rai-robotModels/pr2/meshes/', glob.glob('rai-robotModels/pr2/meshes/*.ply')),
    #     ('rai-robotModels/pr2/meshes/forearm_v0/', glob.glob('rai-robotModels/pr2/meshes/forearm_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/head_v0/', glob.glob('rai-robotModels/pr2/meshes/head_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/shoulder_v0/', glob.glob('rai-robotModels/pr2/meshes/shoulder_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/torso_v0/', glob.glob('rai-robotModels/pr2/meshes/torso_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/base_v0/', glob.glob('rai-robotModels/pr2/meshes/base_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/gripper_v0/', glob.glob('rai-robotModels/pr2/meshes/gripper_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/tilting_laser_v0/', glob.glob('rai-robotModels/pr2/meshes/tilting_laser_v0/*.ply')),
    #     ('rai-robotModels/pr2/meshes/upper_arm_v0/', glob.glob('rai-robotModels/pr2/meshes/upper_arm_v0/*.ply')),
    #     ('rai-robotModels/robotiq/', glob.glob('rai-robotModels/robotiq/*.g')),
    #     ('rai-robotModels/robotiq/meshes/visual/', glob.glob('rai-robotModels/robotiq/meshes/visual/*.ply')),
    #     ('rai-robotModels/scenarios/', glob.glob('rai-robotModels/scenarios/*.g')),
    #     ('rai-robotModels/tests/', glob.glob('rai-robotModels/tests/*.g')),
    # ],
    
    description="Robotic AI basics",
    long_description="Robotic AI basics, see https://github.com/MarcToussaint/rai",
    long_description_content_type="text/markdown",

    distclass=BinaryDistribution,
    version='0.0.9',
    url='https://www.user.tu-berlin.de/mtoussai/',
    author='Marc Toussaint',
    author_email='toussaint@tu-berlin.de',
)
