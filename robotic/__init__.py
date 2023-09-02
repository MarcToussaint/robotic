from . import ry

from .version import __version__
ry.__version__ = __version__

import os
ry.setRaiPath( os.path.abspath(os.path.dirname(__file__)) + '/rai-robotModels' )
