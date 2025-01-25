from ._robotic import *

_left = ArgWord._left
_right = ArgWord._right
_sequence = ArgWord._sequence
_path = ArgWord._path

from .version import __version__

from .manipulation import KOMO_ManipulationHelper

import os
setRaiPath( os.path.abspath(os.path.dirname(__file__)) + '/rai-robotModels' )
