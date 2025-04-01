from setuptools import setup, dist
#from setuptools.command.install import install
from setuptools.command.build_ext import build_ext
import setuptools
import os
import glob
import subprocess
import platform

myversion = {}
with open("robotic/version.py") as fp:
    exec(fp.read(), myversion)

class BinaryDistribution(dist.Distribution):
    def has_ext_modules(foo):
        return True

from pathlib import Path
long_description = (Path(__file__).parent / "README.md").read_text()

setup(
    name='robotic',
    packages=['robotic'],
    #packages=setuptools.find_namespace_packages(),
    package_data={
        'robotic': ['_robotic.so', 'librai.so', 'meshTool', '_robotic.pyi', 'DataGen.pyi', 'version.py', 'manipulation.py', 'render.py', 'nlp.py',
                    'src/*',
                    'rai-robotModels/*/*', 'rai-robotModels/*/*/*',
                    'include/rai/*/*', 'include/rai/*/*/*']
    },
    include_package_data=True,
    # cmdclass={ 'build_ext': CustomCommand },
    scripts=['robotic/ry-view', 'robotic/ry-bot', 'robotic/ry-info', 'robotic/ry-test', 'robotic/ry-meshTool', 'robotic/ry-h5info', 'robotic/ry-urdfConvert.py'],

    description="Robotic Control Interface & Manipulation Planning Library",
    long_description=long_description,
    long_description_content_type="text/markdown",

    #install_requires=[ 'numpy' ],
    
    distclass=BinaryDistribution,
    version=myversion['__version__'],
    url='https://github.com/MarcToussaint/robotic/',
    author='Marc Toussaint',
    author_email='toussaint@tu-berlin.de',
)
