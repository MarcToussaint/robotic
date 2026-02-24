from ._robotic import *

_left = ArgWord._left
_right = ArgWord._right
_sequence = ArgWord._sequence
_path = ArgWord._path

from .manipulation import KOMO_ManipulationHelper

import os
rai_path = os.path.abspath(os.path.dirname(__file__)) + '/rai-robotModels'
os.environ["RAI_PATH"] = rai_path
setRaiPath( rai_path )

def version():
    import importlib.metadata

    return importlib.metadata.version('robotic')
