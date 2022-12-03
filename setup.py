from setuptools import setup, dist
from setuptools.command.install import install
import setuptools
import os
import glob

class BinaryDistribution(dist.Distribution):
    def has_ext_modules(foo):
        return True

#try:
#    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
#    class bdist_wheel(_bdist_wheel):
#        def finalize_options(self):
#            _bdist_wheel.finalize_options(self)
#            self.root_is_pure = False
#except ImportError:
#    bdist_wheel = None
    
setup(
    name='robotic',
    packages=['robotic'],
    package_data={'robotic': ['libry.so']},
    include_package_data=True,

    data_files=[
        ('rai-robotModels/objects/',  glob.glob('rai-robotModels/objects/*.g')),
        ('rai-robotModels/panda/',  glob.glob('rai-robotModels/panda/*.g')),
        ('rai-robotModels/panda/meshes/', glob.glob('rai-robotModels/panda/meshes/*.ply')),
        ('rai-robotModels/pr2/', glob.glob('rai-robotModels/pr2/*.g')),
        ('rai-robotModels/pr2/meshes/', glob.glob('rai-robotModels/pr2/meshes/*.ply')),
        ('rai-robotModels/robotiq/', glob.glob('rai-robotModels/robotiq/*.g')),
        ('rai-robotModels/robotiq/meshes/', glob.glob('rai-robotModels/robotiq/meshes/*.ply')),
        ('rai-robotModels/scenarios/', glob.glob('rai-robotModels/scenarios/*.g')),
        ('rai-robotModels/tests/', glob.glob('rai-robotModels/tests/*.g')),
    ],
    
    description="Robotic AI basics",
    long_description="Robotic AI basics, see https://github.com/MarcToussaint/rai",
    long_description_content_type="text/markdown",

    distclass=BinaryDistribution,
    version='0.0.2',
    url='https://www.user.tu-berlin.de/mtoussai/',
    author='Marc Toussaint',
    author_email='toussaint@tu-berlin.de',
)
