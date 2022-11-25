from setuptools import setup, dist
from setuptools.command.install import install
import setuptools
import os
import glob

# force setuptools to recognize that this is
# actually a binary distribution
class BinaryDistribution(dist.Distribution):
    def has_ext_modules(foo):
        return True

# optional, use README.md as long_description
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

modelfiles = glob.glob('rai-robotModels/**/*.*', recursive=True,)

try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            self.root_is_pure = False
except ImportError:
    bdist_wheel = None
    
setup(
    # this package is called robotic
    name='robotic',

    # this package contains one module,
    # which resides in the subdirectory robotic
    packages=['robotic'],

    # make sure the shared library is included
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
    
    description="This is a short description",
    # optional, the contents of README.md that were read earlier
    long_description=long_description,
    long_description_content_type="text/markdown",

    # See class BinaryDistribution that was defined earlier
    distclass=BinaryDistribution,

    version='0.0.a2',
    url='https://www.user.tu-berlin.de/mtoussai/',
    author='Marc Toussaint',
    author_email='toussaint@tu-berlin.de',
    # ...
)
