from ._robotic import *

_left = ArgWord._left
_right = ArgWord._right
_sequence = ArgWord._sequence
_path = ArgWord._path

from .version import __version__

from .manipulation import KOMO_ManipulationHelper

#from .src.mujoco_io import MujocoLoader
#from .src.config_urdf import URDFLoader
#from .src.mesh_helper import MeshHelper

import os
setRaiPath( os.path.abspath(os.path.dirname(__file__)) + '/rai-robotModels' )
