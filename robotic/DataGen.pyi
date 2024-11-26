"""
rai data generators
"""
from __future__ import annotations
import _robotic
__all__ = ['RndStableConfigs', 'ShapenetGrasps']
class RndStableConfigs:
    """
    A generator of random stable configurations
    """
    def __init__(self) -> None:
        ...
    def getSample(self, config: _robotic.Config, supports: StringA) -> bool:
        """
        sample a random configuration - displayed, access via config passed at construction
        """
    def report(self) -> None:
        """
        info on newton steps -per- feasible sample
        """
    def setOptions(self, verbose: int = 1, frictionCone_mu: float = 0.8) -> RndStableConfigs:
        """
        set options
        """
class ShapenetGrasps:
    """
    A generator of random grasps on random shapenet objects
    """
    def __init__(self) -> None:
        ...
    def displaySamples(self, samples: arr, context: arr, scores: arr = ...) -> None:
        """
        displays all samples
        """
    def evaluateSample(self, sample: arr, context: arr) -> arr:
        """
        returns scores for a single sample - this (row) are numbers where a single 'negative' means fail
        """
    def getSamples(self, nSamples: int) -> tuple[arr, arr, arr]:
        """
        return three arrays: samples X, contexts Z, scores S (each row are scores for one sample - see evaluateSamples)
        """
    def setOptions(self, verbose: int = 1, filesPrefix: ... = 'shapenet/models/', numShapes: int = -1, startShape: int = 0, simVerbose: int = 0, optVerbose: int = 0, simTau: float = 0.01, gripperCloseSpeed: float = 0.001, moveSpeed: float = 0.005, pregraspNormalSdv: float = 0.2) -> ShapenetGrasps:
        """
        set options
        """
    def setPhysxOptions(self, verbose: int = 1, yGravity: bool = False, softBody: bool = False, multiBody: bool = True, multiBodyDisableGravity: bool = True, jointedBodies: bool = False, angularDamping: float = 0.1, defaultFriction: float = 1.0, defaultRestitution: float = 0.1, motorKp: float = 1000.0, motorKd: float = 100.0, gripperKp: float = 10000.0, gripperKd: float = 100.0) -> ShapenetGrasps:
        """
        set options
        """
