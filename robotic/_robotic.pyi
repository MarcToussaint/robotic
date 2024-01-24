"""rai bindings"""
from __future__ import annotations
import _robotic
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "ArgWord",
    "BotOp",
    "CameraView",
    "CameraViewSensor",
    "Ceres",
    "Config",
    "ConfigurationViewer",
    "ControlMode",
    "FS",
    "Feature",
    "Frame",
    "ImageViewer",
    "ImpType",
    "Ipopt",
    "JT",
    "KOMO",
    "KOMO_Objective",
    "LBFGS",
    "NLP",
    "NLP_Factory",
    "NLP_Solver",
    "NLP_SolverID",
    "NLP_SolverOptions",
    "NLopt",
    "OT",
    "OptBench_Skeleton_Handover",
    "OptBench_Skeleton_Pick",
    "OptBench_Skeleton_StackAndBalance",
    "OptBenchmark_InvKin_Endeff",
    "PathFinder",
    "PointCloudViewer",
    "ST",
    "SY",
    "Simulation",
    "SimulationEngine",
    "Skeleton",
    "SolverReturn",
    "XBall",
    "above",
    "aboveBox",
    "acceleration",
    "accumulatedCollisions",
    "adversarialDropper",
    "alignByInt",
    "angularVel",
    "augmentedLag",
    "bounce",
    "box",
    "boxGraspX",
    "boxGraspY",
    "boxGraspZ",
    "break",
    "bullet",
    "camera",
    "capsule",
    "closeGripper",
    "compiled",
    "contact",
    "contactComplementary",
    "contactConstraints",
    "contactStick",
    "cylinder",
    "dampMotion",
    "depthImage2PointCloud",
    "depthNoise",
    "distance",
    "downUp",
    "dynamic",
    "dynamicOn",
    "dynamicTrans",
    "end",
    "energy",
    "eq",
    "f",
    "forceBalance",
    "free",
    "gazeAt",
    "generic",
    "getStartGoalPath",
    "gradientDescent",
    "hingeX",
    "hingeY",
    "hingeZ",
    "identical",
    "ineq",
    "ineqB",
    "ineqP",
    "inside",
    "insideBox",
    "jointLimits",
    "jointState",
    "kinematic",
    "lift",
    "logBarrier",
    "magic",
    "magicTrans",
    "makeFree",
    "marker",
    "mesh",
    "moveGripper",
    "negDistance",
    "newton",
    "noPenetrations",
    "none",
    "objectImpulses",
    "oppose",
    "pairCollision_negScalar",
    "pairCollision_normal",
    "pairCollision_p1",
    "pairCollision_p2",
    "pairCollision_vector",
    "params_add",
    "params_clear",
    "params_file",
    "params_print",
    "phiTransXY",
    "physics",
    "physx",
    "pointCloud",
    "pose",
    "poseDiff",
    "poseEq",
    "poseRel",
    "position",
    "positionDiff",
    "positionEq",
    "positionRel",
    "push",
    "pushAndPlace",
    "qItself",
    "quad",
    "quasiStatic",
    "quasiStaticOn",
    "quatBall",
    "quaternion",
    "quaternionDiff",
    "quaternionRel",
    "raiPath",
    "relPosY",
    "restingOn",
    "rgbNoise",
    "rigid",
    "rprop",
    "scalarProductXX",
    "scalarProductXY",
    "scalarProductXZ",
    "scalarProductYX",
    "scalarProductYY",
    "scalarProductYZ",
    "scalarProductZZ",
    "sdf",
    "setRaiPath",
    "singleSquaredPenalty",
    "sos",
    "sphere",
    "spline",
    "squaredPenalty",
    "ssBox",
    "ssBoxElip",
    "ssCvx",
    "ssCylinder",
    "stable",
    "stableOn",
    "stableOnX",
    "stableOnY",
    "stablePose",
    "stableRelPose",
    "stableYPhi",
    "stableZero",
    "standingAbove",
    "tau",
    "test",
    "topBoxGrasp",
    "topBoxPlace",
    "touch",
    "touchBoxNormalX",
    "touchBoxNormalY",
    "touchBoxNormalZ",
    "trans3",
    "transAccelerations",
    "transVelocities",
    "transX",
    "transXY",
    "transXYPhi",
    "transY",
    "transYPhi",
    "transZ",
    "universal",
    "vectorX",
    "vectorXDiff",
    "vectorXRel",
    "vectorY",
    "vectorYDiff",
    "vectorYRel",
    "vectorZ",
    "vectorZDiff",
    "vectorZRel",
    "velocity"
]


class ArgWord():
    """
    Members:

      _left

      _right

      _sequence

      _path
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'_left': <ArgWord._left: 0>, '_right': <ArgWord._right: 1>, '_sequence': <ArgWord._sequence: 2>, '_path': <ArgWord._path: 3>}
    _left: _robotic.ArgWord # value = <ArgWord._left: 0>
    _path: _robotic.ArgWord # value = <ArgWord._path: 3>
    _right: _robotic.ArgWord # value = <ArgWord._right: 1>
    _sequence: _robotic.ArgWord # value = <ArgWord._sequence: 2>
    pass
class BotOp():
    """
    Robot Operation interface -- see https://marctoussaint.github.io/robotics-course/tutorials/1b-botop.html
    """
    def __init__(self, C: Config, useRealRobot: bool) -> None: 
        """
        constructor
        """
    def getCameraFxycxy(self, sensorName: str) -> arr: 
        """
        returns camera intrinsics
        """
    def getImageAndDepth(self, sensorName: str) -> tuple: 
        """
        returns image and depth from a camera sensor
        """
    def getImageDepthPcl(self, sensorName: str, globalCoordinates: bool = False) -> tuple: 
        """
        returns image, depth and point cloud (assuming sensor knows intrinsics) from a camera sensor, optionally in global instead of camera-frame-relative coordinates
        """
    def getKeyPressed(self) -> int: 
        """
        get key pressed in window at last sync
        """
    def getTimeToEnd(self) -> float: 
        """
        get time-to-go of the current spline reference that is tracked (use getTimeToEnd()<=0. to check if motion execution is done)
        """
    def get_q(self) -> arr: 
        """
        get the current (real) robot joint vector
        """
    def get_qDot(self) -> arr: 
        """
        get the current (real) robot joint velocities
        """
    def get_qHome(self) -> arr: 
        """
        returns home joint vector (defined as the configuration C when you created BotOp)
        """
    def get_t(self) -> float: 
        """
        returns the control time (absolute time managed by the high freq tracking controller)
        """
    def get_tauExternal(self) -> arr: 
        """
        get the current (real) robot joint torques (external: gravity & acceleration removed) -- each call averages from last call; first call might return nonsense!
        """
    def gripperClose(self, leftRight: ArgWord, force: float = 10.0, width: float = 0.05, speed: float = 0.1) -> None: 
        """
        close gripper
        """
    def gripperCloseGrasp(self, leftRight: ArgWord, objName: str, force: float = 10.0, width: float = 0.05, speed: float = 0.1) -> None: 
        """
        close gripper and indicate what should be grasped -- makes no different in real, but helps simulation to mimic grasping more reliably
        """
    def gripperDone(self, leftRight: ArgWord) -> bool: 
        """
        returns if gripper is done
        """
    def gripperMove(self, leftRight: ArgWord, width: float = 0.075, speed: float = 0.2) -> None: 
        """
        move the gripper to width (default: open)
        """
    def gripperPos(self, leftRight: ArgWord) -> float: 
        """
        returns the gripper pos
        """
    def hold(self, floating: bool = False, damping: bool = True) -> None: 
        """
        hold the robot with a trivial PD controller, floating means reference = real, without damping the robot is free floating
        """
    def home(self, C: Config) -> None: 
        """
        immediately drive the robot home (see get_qHome); keeps argument C synced; same as moveTo(qHome, 1., True); wait(C);
        """
    def move(self, path: arr, times: arr, overwrite: bool = False, overwriteCtrlTime: float = -1.0) -> None: 
        """
        core motion command: set a spline motion reference; if only a single time [T] is given for multiple waypoints, it assumes equal time spacing with TOTAL time T

        By default, the given spline is APPENDED to the current reference spline. The user can also enforce the given spline to overwrite the current reference starting at the given absolute ctrlTime. This allows implementation of reactive (e.g. MPC-style) control. However, the user needs to take care that overwriting is done in a smooth way, i.e., that the given spline starts with a pos/vel that is close to the pos/vel of the current reference at the given ctrlTime.
        """
    def moveAutoTimed(self, path: arr, maxVel: float = 1.0, maxAcc: float = 1.0) -> None: 
        """
        helper to execute a path (typically fine resolution, from KOMO or RRT) with equal time spacing chosen for given max vel/acc
        """
    def moveTo(self, q_target: arr, timeCost: float = 1.0, overwrite: bool = False) -> None: 
        """
        helper to move to a single joint vector target, where timing is chosen optimally based on the given timing cost

        When using overwrite, this immediately steers to the target -- use this as a well-timed reactive q_target controller
        """
    def setCompliance(self, J: arr, compliance: float = 0.5) -> None: 
        """
        set a task space compliant, where J defines the task space Jacobian, and compliance goes from 0 (no compliance) to 1 (full compliance, but still some damping)
        """
    def setControllerWriteData(self, arg0: int) -> None: 
        """
        [for internal debugging only] triggers writing control data into a file
        """
    def stop(self, C: Config) -> None: 
        """
        immediately stop the robot; keeps argument C synced; same as moveTo(get_q(), 1., True); wait(C);
        """
    def sync(self, C: Config, waitTime: float = 0.1) -> int: 
        """
        sync your workspace configuration C with the robot state
        """
    def wait(self, C: Config, forKeyPressed: bool = True, forTimeToEnd: bool = True) -> int: 
        """
        repeatedly sync your workspace C until a key is pressed or motion ends (optionally)
        """
    pass
class CameraView():
    """
    Offscreen rendering
    """
    def __init__(self, config: Config, offscreen: bool = True) -> None: 
        """
        constructor
        """
    def computeImageAndDepth(self, config: Config, visualsOnly: bool = True) -> tuple: 
        """
        returns image and depth from a camera sensor; the 'config' argument needs to be the same configuration as in the constructor, but in new state
        """
    @staticmethod
    def computeSegmentationID(*args, **kwargs) -> typing.Any: 
        """
        return a uint16 array with object ID segmentation
        """
    @staticmethod
    def computeSegmentationImage(*args, **kwargs) -> typing.Any: 
        """
        return an rgb image encoding the object ID segmentation
        """
    def getFxycxy(self) -> arr: 
        """
        return the camera intrinsics f_x, f_y, c_x, c_y
        """
    @staticmethod
    def setCamera(*args, **kwargs) -> typing.Any: 
        """
        select a camera, typically a frame that has camera info attributes
        """
    pass
class CameraViewSensor():
    pass
class Config():
    """
    Core data structure to represent a kinematic configuration (essentially a tree of frames). See https://marctoussaint.github.io/robotics-course/tutorials/1a-configurations.html
    """
    def __init__(self) -> None: 
        """
        initializes to an empty configuration, with no frames
        """
    def addConfigurationCopy(self, config: Config, tau: float = 1.0) -> None: ...
    def addFile(self, filename: str, namePrefix: str = '') -> Frame: 
        """
        add the contents of the file to C
        """
    def addFrame(self, name: str, parent: str = '', args: str = '') -> Frame: 
        """
        add a new frame to C; optionally make this a child to the given parent; use the Frame methods to set properties of the new frame
        """
    def animate(self) -> None: 
        """
        displays while articulating all dofs in a row
        """
    def attach(self, arg0: str, arg1: str) -> None: 
        """
        change the configuration by creating a rigid joint from frame1 to frame2, adopting their current relative pose. This also breaks the first joint that is parental to frame2 and reverses the topological order from frame2 to the broken joint
        """
    def clear(self) -> None: 
        """
        clear all frames and additional data; becomes the empty configuration, with no frames
        """
    def computeCollisions(self) -> None: 
        """
        call the broadphase collision engine (SWIFT++ or FCL) to generate the list of collisions (or near proximities) between all frame shapes that have the collision tag set non-zero
        """
    def copy(self, C2: Config) -> None: 
        """
        make C a (deep) copy of the given C2
        """
    def delFrame(self, frameName: str) -> None: 
        """
        destroy and remove a frame from C
        """
    def equationOfMotion(self, qdot: typing.List[float], gravity: bool) -> tuple: ...
    def eval(self, featureSymbol: FS, frames: StringA = [], scale: arr = array(1.e-05), target: arr = array(0.0078125), order: int = -1) -> tuple: 
        """
        evaluate a feature -- see https://marctoussaint.github.io/robotics-course/tutorials/features.html
        """
    def feature(self, featureSymbol: FS, frameNames: typing.List[str] = [], scale: typing.List[float] = [], target: typing.List[float] = [], order: int = -1) -> Feature: 
        """
        create a feature (a differentiable map from joint state to a vector space), as they're typically used for IK or optimization. See the dedicated tutorial for details. featureSymbol defines which mapping this is (position, vectors, collision distance, etc). many mapping refer to one or several frames, which need to be specified using frameNames
        """
    def frame(self, frameName: str, warnIfNotExist: bool = True) -> Frame: 
        """
        get access to a frame by name; use the Frame methods to set/get frame properties
        """
    def frames(self) -> typing.List[Frame]: ...
    def getCollisions(self, belowMargin: float = 1.0) -> list: 
        """
        return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance). Optionally report only on distances below a margin To get really precise distances and penetrations use the FS.distance feature with the two frame names
        """
    def getDofIDs(self) -> typing.List[int]: ...
    def getFrame(self, frameName: str, warnIfNotExist: bool = True) -> Frame: 
        """
        get access to a frame by name; use the Frame methods to set/get frame properties
        """
    def getFrameDimension(self) -> int: 
        """
        get the total number of frames
        """
    def getFrameNames(self) -> typing.List[str]: 
        """
        get the list of frame names
        """
    @typing.overload
    def getFrameState(self) -> numpy.ndarray[numpy.float64]: 
        """
        get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame

        TODO remove -> use individual frame!
        """
    @typing.overload
    def getFrameState(self, arg0: str) -> numpy.ndarray[numpy.float64]: ...
    def getJointDimension(self) -> int: 
        """
        get the total number of degrees of freedom
        """
    def getJointLimits(self) -> arr: 
        """
        get the joint limits as a n-by-2 matrix; for dofs that do not have limits defined, the entries are [0,-1] (i.e. upper limit < lower limit)
        """
    def getJointNames(self) -> StringA: 
        """
        get the list of joint names
        """
    def getJointState(self) -> arr: 
        """
        get the joint state as a numpy vector, optionally only for a subset of joints specified as list of joint names
        """
    def getTotalPenetration(self) -> float: 
        """
        returns the sum of all penetrations
        """
    def makeObjectsConvex(self) -> None: 
        """
        remake all meshes associated with all frames to become their convex hull
        """
    def report(self) -> str: ...
    def selectJoints(self, jointNames: typing.List[str], notThose: bool = False) -> None: 
        """
        redefine what are considered the DOFs of this configuration: only joints listed in jointNames are considered part of the joint state and define the number of DOFs
        """
    @typing.overload
    def setFrameState(self, X: typing.List[float], frames: typing.List[str] = []) -> None: 
        """
        set the frame state, optionally only for a subset of frames specified as list of frame names

        set the frame state, optionally only for a subset of frames specified as list of frame names
        """
    @typing.overload
    def setFrameState(self, X: numpy.ndarray, frames: typing.List[str] = []) -> None: ...
    def setJointState(self, q: arr, joints: list = []) -> None: 
        """
        set the joint state, optionally only for a subset of joints specified as list of joint names
        """
    def setJointStateSlice(self, arg0: typing.List[float], arg1: int) -> None: ...
    def sortFrames(self) -> None: 
        """
        resort the internal order of frames according to the tree topology. This is important before saving the configuration.
        """
    def stepDynamics(self, qdot: typing.List[float], u_control: typing.List[float], tau: float, dynamicNoise: float, gravity: bool) -> numpy.ndarray[numpy.float64]: ...
    def view(self, pause: bool = False, message: str = None) -> int: 
        """
        open a view window for the configuration
        """
    def view_close(self) -> None: 
        """
        close the view
        """
    def view_focalLength(self) -> float: 
        """
        return the focal length of the view camera (only intrinsic parameter)
        """
    def view_fxycxy(self) -> arr: ...
    def view_getDepth(self) -> numpy.ndarray[numpy.float32]: ...
    def view_getRgb(self) -> numpy.ndarray[numpy.uint8]: ...
    def view_playVideo(self, delay: float = 1.0, saveVideoPath: str = None) -> None: ...
    def view_pose(self) -> arr: 
        """
        return the 7D pose of the view camera
        """
    def view_raise(self) -> None: 
        """
        raise the view
        """
    def view_recopyMeshes(self) -> None: ...
    def view_savePng(self, pathPrefix: str = 'z.vid/') -> None: 
        """
        saves a png image of the current view, numbered with a global counter, with the intention to make a video
        """
    def view_setCamera(self, arg0: Frame) -> None: 
        """
        set the camera pose to a frame, and check frame attributes for intrinsic parameters (focalLength, width height)
        """
    def watchFile(self, arg0: str) -> None: 
        """
        launch a viewer that listents (inode) to changes of a file (made by you in an editor), and reloads, displays and animates the configuration whenever the file is changed
        """
    def write(self) -> str: 
        """
        write the full configuration in a string (roughly yaml), e.g. for file export
        """
    def writeCollada(self, filename: str, format: str = 'collada') -> None: 
        """
        write the full configuration in a collada file for export
        """
    def writeMesh(self, filename: str) -> None: 
        """
        write the full configuration in a ply mesh file
        """
    def writeMeshes(self, pathPrefix: str) -> None: 
        """
        write all object meshes in a directory
        """
    def writeURDF(self) -> str: 
        """
        write the full configuration as URDF in a string, e.g. for file export
        """
    pass
class ConfigurationViewer():
    pass
class ControlMode():
    """
    Members:

      none

      position

      velocity

      acceleration

      spline
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'none': <ControlMode.none: 0>, 'position': <ControlMode.position: 1>, 'velocity': <ControlMode.velocity: 2>, 'acceleration': <ControlMode.acceleration: 3>, 'spline': <ControlMode.spline: 5>}
    acceleration: _robotic.ControlMode # value = <ControlMode.acceleration: 3>
    none: _robotic.ControlMode # value = <ControlMode.none: 0>
    position: _robotic.ControlMode # value = <ControlMode.position: 1>
    spline: _robotic.ControlMode # value = <ControlMode.spline: 5>
    velocity: _robotic.ControlMode # value = <ControlMode.velocity: 2>
    pass
class FS():
    """
    Members:

      position

      positionDiff

      positionRel

      quaternion

      quaternionDiff

      quaternionRel

      pose

      poseDiff

      poseRel

      vectorX

      vectorXDiff

      vectorXRel

      vectorY

      vectorYDiff

      vectorYRel

      vectorZ

      vectorZDiff

      vectorZRel

      scalarProductXX

      scalarProductXY

      scalarProductXZ

      scalarProductYX

      scalarProductYY

      scalarProductYZ

      scalarProductZZ

      gazeAt

      angularVel

      accumulatedCollisions

      jointLimits

      distance

      negDistance

      oppose

      qItself

      jointState

      aboveBox

      insideBox

      pairCollision_negScalar

      pairCollision_vector

      pairCollision_normal

      pairCollision_p1

      pairCollision_p2

      standingAbove

      physics

      contactConstraints

      energy

      transAccelerations

      transVelocities
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'position': <FS.position: 0>, 'positionDiff': <FS.positionDiff: 1>, 'positionRel': <FS.positionRel: 2>, 'quaternion': <FS.quaternion: 3>, 'quaternionDiff': <FS.quaternionDiff: 4>, 'quaternionRel': <FS.quaternionRel: 5>, 'pose': <FS.pose: 6>, 'poseDiff': <FS.poseDiff: 7>, 'poseRel': <FS.poseRel: 8>, 'vectorX': <FS.vectorX: 9>, 'vectorXDiff': <FS.vectorXDiff: 10>, 'vectorXRel': <FS.vectorXRel: 11>, 'vectorY': <FS.vectorY: 12>, 'vectorYDiff': <FS.vectorYDiff: 13>, 'vectorYRel': <FS.vectorYRel: 14>, 'vectorZ': <FS.vectorZ: 15>, 'vectorZDiff': <FS.vectorZDiff: 16>, 'vectorZRel': <FS.vectorZRel: 17>, 'scalarProductXX': <FS.scalarProductXX: 18>, 'scalarProductXY': <FS.scalarProductXY: 19>, 'scalarProductXZ': <FS.scalarProductXZ: 20>, 'scalarProductYX': <FS.scalarProductYX: 21>, 'scalarProductYY': <FS.scalarProductYY: 22>, 'scalarProductYZ': <FS.scalarProductYZ: 23>, 'scalarProductZZ': <FS.scalarProductZZ: 24>, 'gazeAt': <FS.gazeAt: 25>, 'angularVel': <FS.angularVel: 26>, 'accumulatedCollisions': <FS.accumulatedCollisions: 27>, 'jointLimits': <FS.jointLimits: 28>, 'distance': <FS.distance: 29>, 'negDistance': <FS.distance: 29>, 'oppose': <FS.oppose: 30>, 'qItself': <FS.qItself: 31>, 'jointState': <FS.qItself: 31>, 'aboveBox': <FS.aboveBox: 33>, 'insideBox': <FS.insideBox: 34>, 'pairCollision_negScalar': <FS.pairCollision_negScalar: 35>, 'pairCollision_vector': <FS.pairCollision_vector: 36>, 'pairCollision_normal': <FS.pairCollision_normal: 37>, 'pairCollision_p1': <FS.pairCollision_p1: 38>, 'pairCollision_p2': <FS.pairCollision_p2: 39>, 'standingAbove': <FS.standingAbove: 40>, 'physics': <FS.physics: 41>, 'contactConstraints': <FS.contactConstraints: 42>, 'energy': <FS.energy: 43>, 'transAccelerations': <FS.transAccelerations: 44>, 'transVelocities': <FS.transVelocities: 45>}
    aboveBox: _robotic.FS # value = <FS.aboveBox: 33>
    accumulatedCollisions: _robotic.FS # value = <FS.accumulatedCollisions: 27>
    angularVel: _robotic.FS # value = <FS.angularVel: 26>
    contactConstraints: _robotic.FS # value = <FS.contactConstraints: 42>
    distance: _robotic.FS # value = <FS.distance: 29>
    energy: _robotic.FS # value = <FS.energy: 43>
    gazeAt: _robotic.FS # value = <FS.gazeAt: 25>
    insideBox: _robotic.FS # value = <FS.insideBox: 34>
    jointLimits: _robotic.FS # value = <FS.jointLimits: 28>
    jointState: _robotic.FS # value = <FS.qItself: 31>
    negDistance: _robotic.FS # value = <FS.distance: 29>
    oppose: _robotic.FS # value = <FS.oppose: 30>
    pairCollision_negScalar: _robotic.FS # value = <FS.pairCollision_negScalar: 35>
    pairCollision_normal: _robotic.FS # value = <FS.pairCollision_normal: 37>
    pairCollision_p1: _robotic.FS # value = <FS.pairCollision_p1: 38>
    pairCollision_p2: _robotic.FS # value = <FS.pairCollision_p2: 39>
    pairCollision_vector: _robotic.FS # value = <FS.pairCollision_vector: 36>
    physics: _robotic.FS # value = <FS.physics: 41>
    pose: _robotic.FS # value = <FS.pose: 6>
    poseDiff: _robotic.FS # value = <FS.poseDiff: 7>
    poseRel: _robotic.FS # value = <FS.poseRel: 8>
    position: _robotic.FS # value = <FS.position: 0>
    positionDiff: _robotic.FS # value = <FS.positionDiff: 1>
    positionRel: _robotic.FS # value = <FS.positionRel: 2>
    qItself: _robotic.FS # value = <FS.qItself: 31>
    quaternion: _robotic.FS # value = <FS.quaternion: 3>
    quaternionDiff: _robotic.FS # value = <FS.quaternionDiff: 4>
    quaternionRel: _robotic.FS # value = <FS.quaternionRel: 5>
    scalarProductXX: _robotic.FS # value = <FS.scalarProductXX: 18>
    scalarProductXY: _robotic.FS # value = <FS.scalarProductXY: 19>
    scalarProductXZ: _robotic.FS # value = <FS.scalarProductXZ: 20>
    scalarProductYX: _robotic.FS # value = <FS.scalarProductYX: 21>
    scalarProductYY: _robotic.FS # value = <FS.scalarProductYY: 22>
    scalarProductYZ: _robotic.FS # value = <FS.scalarProductYZ: 23>
    scalarProductZZ: _robotic.FS # value = <FS.scalarProductZZ: 24>
    standingAbove: _robotic.FS # value = <FS.standingAbove: 40>
    transAccelerations: _robotic.FS # value = <FS.transAccelerations: 44>
    transVelocities: _robotic.FS # value = <FS.transVelocities: 45>
    vectorX: _robotic.FS # value = <FS.vectorX: 9>
    vectorXDiff: _robotic.FS # value = <FS.vectorXDiff: 10>
    vectorXRel: _robotic.FS # value = <FS.vectorXRel: 11>
    vectorY: _robotic.FS # value = <FS.vectorY: 12>
    vectorYDiff: _robotic.FS # value = <FS.vectorYDiff: 13>
    vectorYRel: _robotic.FS # value = <FS.vectorYRel: 14>
    vectorZ: _robotic.FS # value = <FS.vectorZ: 15>
    vectorZDiff: _robotic.FS # value = <FS.vectorZDiff: 16>
    vectorZRel: _robotic.FS # value = <FS.vectorZRel: 17>
    pass
class Feature():
    """
    to be removed - use only directly from Config
    """
    def description(self, arg0: Config) -> str: ...
    def eval(self, arg0: Config) -> tuple: ...
    def setOrder(self, arg0: int) -> Feature: ...
    def setScale(self, arg0: arr) -> Feature: ...
    def setTarget(self, arg0: arr) -> Feature: ...
    pass
class Frame():
    """
    A (coordinate) of a configuration, which can have a parent, and associated shape, joint, and/or inertia
    """
    def addAttributes(self, arg0: dict) -> None: 
        """
        add/set attributes for the frame
        """
    def getAttributes(self) -> dict: 
        """
        get frame attributes
        """
    def getJointState(self) -> arr: ...
    def getMeshPoints(self) -> arr: ...
    def getMeshTriangles(self) -> uintA: ...
    def getPosition(self) -> arr: ...
    def getQuaternion(self) -> arr: ...
    def getRelativePosition(self) -> arr: ...
    def getRelativeQuaternion(self) -> arr: ...
    def getRotationMatrix(self) -> arr: ...
    def getSize(self) -> arr: ...
    def info(self) -> dict: ...
    def setAttribute(self, arg0: str, arg1: float) -> Frame: ...
    def setColor(self, arg0: arr) -> Frame: ...
    def setContact(self, arg0: int) -> Frame: ...
    @staticmethod
    def setConvexMesh(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def setDensity(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def setImplicitSurface(*args, **kwargs) -> typing.Any: ...
    def setJoint(self, arg0: JT) -> Frame: ...
    def setJointState(self, arg0: arr) -> Frame: ...
    def setMass(self, arg0: float) -> Frame: ...
    def setMesh(self, vertices: arr, triangles: uintA, colors: arr) -> None: 
        """
        set mesh
        """
    def setMeshAsLines(self, arg0: typing.List[float]) -> None: ...
    def setParent(self, parent: Frame, keepAbsolutePose_and_adaptRelativePose: bool = False, checkForLoop: bool = False) -> Frame: ...
    def setPointCloud(self, points: numpy.ndarray, colors: numpy.ndarray[numpy.uint8] = array([], dtype=uint8)) -> None: ...
    def setPose(self, arg0: str) -> None: ...
    def setPosition(self, arg0: arr) -> Frame: ...
    def setQuaternion(self, arg0: arr) -> Frame: ...
    def setRelativePose(self, arg0: str) -> None: ...
    def setRelativePosition(self, arg0: arr) -> Frame: ...
    def setRelativeQuaternion(self, arg0: arr) -> Frame: ...
    def setShape(self, type: ST, size: arr) -> Frame: ...
    def unLink(self) -> Frame: ...
    @property
    def name(self) -> rai::String:
        """
        :type: rai::String
        """
    @name.setter
    def name(self, arg0: rai::String) -> None:
        pass
    pass
class ImageViewer():
    pass
class ImpType():
    """
    Members:

      closeGripper

      moveGripper

      depthNoise

      rgbNoise

      adversarialDropper

      objectImpulses

      noPenetrations
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'closeGripper': <ImpType.closeGripper: 0>, 'moveGripper': <ImpType.moveGripper: 1>, 'depthNoise': <ImpType.depthNoise: 2>, 'rgbNoise': <ImpType.rgbNoise: 3>, 'adversarialDropper': <ImpType.adversarialDropper: 4>, 'objectImpulses': <ImpType.objectImpulses: 5>, 'noPenetrations': <ImpType.noPenetrations: 7>}
    adversarialDropper: _robotic.ImpType # value = <ImpType.adversarialDropper: 4>
    closeGripper: _robotic.ImpType # value = <ImpType.closeGripper: 0>
    depthNoise: _robotic.ImpType # value = <ImpType.depthNoise: 2>
    moveGripper: _robotic.ImpType # value = <ImpType.moveGripper: 1>
    noPenetrations: _robotic.ImpType # value = <ImpType.noPenetrations: 7>
    objectImpulses: _robotic.ImpType # value = <ImpType.objectImpulses: 5>
    rgbNoise: _robotic.ImpType # value = <ImpType.rgbNoise: 3>
    pass
class JT():
    """
    Members:

      hingeX

      hingeY

      hingeZ

      transX

      transY

      transZ

      transXY

      trans3

      transXYPhi

      transYPhi

      universal

      rigid

      quatBall

      phiTransXY

      XBall

      free

      generic

      tau
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    XBall: _robotic.JT # value = <JT.XBall: 15>
    __members__: dict # value = {'hingeX': <JT.hingeX: 1>, 'hingeY': <JT.hingeY: 2>, 'hingeZ': <JT.hingeZ: 3>, 'transX': <JT.transX: 4>, 'transY': <JT.transY: 5>, 'transZ': <JT.transZ: 6>, 'transXY': <JT.transXY: 7>, 'trans3': <JT.trans3: 8>, 'transXYPhi': <JT.transXYPhi: 9>, 'transYPhi': <JT.transYPhi: 10>, 'universal': <JT.universal: 11>, 'rigid': <JT.rigid: 12>, 'quatBall': <JT.quatBall: 13>, 'phiTransXY': <JT.phiTransXY: 14>, 'XBall': <JT.XBall: 15>, 'free': <JT.free: 16>, 'generic': <JT.generic: 17>, 'tau': <JT.tau: 18>}
    free: _robotic.JT # value = <JT.free: 16>
    generic: _robotic.JT # value = <JT.generic: 17>
    hingeX: _robotic.JT # value = <JT.hingeX: 1>
    hingeY: _robotic.JT # value = <JT.hingeY: 2>
    hingeZ: _robotic.JT # value = <JT.hingeZ: 3>
    phiTransXY: _robotic.JT # value = <JT.phiTransXY: 14>
    quatBall: _robotic.JT # value = <JT.quatBall: 13>
    rigid: _robotic.JT # value = <JT.rigid: 12>
    tau: _robotic.JT # value = <JT.tau: 18>
    trans3: _robotic.JT # value = <JT.trans3: 8>
    transX: _robotic.JT # value = <JT.transX: 4>
    transXY: _robotic.JT # value = <JT.transXY: 7>
    transXYPhi: _robotic.JT # value = <JT.transXYPhi: 9>
    transY: _robotic.JT # value = <JT.transY: 5>
    transYPhi: _robotic.JT # value = <JT.transYPhi: 10>
    transZ: _robotic.JT # value = <JT.transZ: 6>
    universal: _robotic.JT # value = <JT.universal: 11>
    pass
class KOMO():
    """
    Constrained solver to optimize configurations or paths. (KOMO = k-order Markov Optimization) -- see https://marctoussaint.github.io/robotics-course/tutorials/1c-komo.html
    """
    @typing.overload
    def __init__(self) -> None: 
        """
        [deprecated] please use the other constructor

        constructor
        * config: the configuration, which is copied once (for IK) or many times (for waypoints/paths) to be the optimization variable
        * phases: the number P of phases (which essentially defines the real-valued interval [0,P] over which objectives can be formulated)
        * slicesPerPhase: the discretizations per phase -> in total we have phases*slicesPerPhases configurations which form the path and over which we optimize
        * kOrder: the 'Markov-order', i.e., maximal tuple of configurations over which we formulate features (e.g. take finite differences)
        * enableCollisions: if True, KOMO runs a broadphase collision check (using libFCL) in each optimization step -- only then accumulative collision/penetration features will correctly evaluate to non-zero. But this is costly.
        """
    @typing.overload
    def __init__(self, config: Config, phases: float, slicesPerPhase: int, kOrder: int, enableCollisions: bool) -> None: ...
    def addControlObjective(self, times: arr, order: int, scale: float = 1.0, target: arr = array(0.0078125), deltaFromSlice: int = 0, deltaToSlice: int = 0) -> Objective: 
        """
        * times: (as for `addObjective`) the phase-interval in which this objective holds; [] means all times
        * order: Do we penalize the jointState directly (order=0: penalizing sqr distance to qHome, order=1: penalizing sqr distances between consecutive configurations (velocities), order=2: penalizing accelerations across 3 configurations)
        * scale: as usual, but modulated by a factor 'sqrt(delta t)' that somehow ensures total control costs in approximately independent of the choice of stepsPerPhase
        """
    @staticmethod
    def addInteraction_elasticBounce(*args, **kwargs) -> typing.Any: ...
    def addModeSwitch(self, times: arr, newMode: SY, frames: StringA, firstSwitch: bool = True) -> None: ...
    def addObjective(self, times: arr, feature: FS, frames: StringA, type: ObjectiveType, scale: arr = array(0.0078125), target: arr = array(0.0078125), order: int = -1) -> None: 
        """
        central method to define objectives in the KOMO NLP:
        * times: the time intervals (subset of configurations in a path) over which this feature is active (irrelevant for IK)
        * feature: the feature symbol (see advanced `Feature` tutorial)
        * frames: the frames for which the feature is computed, given as list of frame names
        * type: whether this is a sum-of-squares (sos) cost, or eq or ineq constraint
        * scale: the matrix(!) by which the feature is multiplied
        * target: the offset which is substracted from the feature (before scaling)
        """
    def addQuaternionNorms(self, times: arr = array(0.0078125), scale: float = 3.0, hard: bool = True) -> None: ...
    def addTimeOptimization(self) -> None: ...
    def clearObjectives(self) -> None: ...
    def getFeatureNames(self) -> StringA: 
        """
        returns a long list of features (per time slice!), to be used by an NLP_Solver
        """
    def getForceInteractions(self) -> list: ...
    def getFrameState(self, arg0: int) -> arr: ...
    def getPath(self) -> arr: ...
    def getPathFrames(self) -> arr: ...
    def getPathTau(self) -> arr: ...
    def getPath_qAll(self) -> arrA: ...
    def getT(self) -> int: ...
    def initOrg(self) -> None: ...
    def initPhaseWithDofsPath(self, t_phase: int, dofIDs: uintA, path: arr, autoResamplePath: bool = False) -> None: ...
    def initRandom(self, verbose: int = 0) -> None: ...
    def initWithConstant(self, q: arr) -> None: ...
    def initWithPath_qOrg(self, q: arr) -> None: ...
    def initWithWaypoints(self, waypoints: arrA, waypointSlicesPerPhase: int = 1, interpolate: bool = False, verbose: int = -1) -> uintA: ...
    def nlp(self) -> NLP: 
        """
        return the problem NLP
        """
    @staticmethod
    def report(*args, **kwargs) -> typing.Any: 
        """
        returns a dict with full list of features, optionally also on problem specs and plotting costs/violations over time
        """
    def reportProblem(self) -> str: ...
    def setConfig(self, config: Config, enableCollisions: bool) -> None: 
        """
        [deprecated] please set directly in constructor
        """
    def setTiming(self, phases: float, slicesPerPhase: int, durationPerPhase: float, kOrder: int) -> None: 
        """
        [deprecated] please set directly in constructor
        """
    def updateRootObjects(self, config: Config) -> None: 
        """
        update root frames (without parents) within all KOMO configurations
        """
    def view(self, pause: bool = False, txt: str = None) -> int: ...
    def view_close(self) -> None: ...
    def view_play(self, pause: bool = False, delay: float = 0.1, saveVideoPath: str = None) -> int: ...
    pass
class KOMO_Objective():
    pass
class NLP():
    """
    Representation of a Nonlinear Mathematical Program
    """
    def evaluate(self, arg0: arr) -> typing.Tuple[arr, arr]: 
        """
        query the NLP at a point $x$; returns the tuple $(phi,J)$, which is the feature vector and its Jacobian; features define cost terms, sum-of-square (sos) terms, inequalities, and equalities depending on 'getFeatureTypes'
        """
    def getBounds(self) -> typing.Tuple[arr, arr]: 
        """
        returns the tuple $(b_{lo},b_{up})$, where both vectors are of same dimensionality of $x$ (or size zero, if there are no bounds)
        """
    def getDimension(self) -> int: 
        """
        return the dimensionality of $x$
        """
    def getFHessian(self, arg0: arr) -> arr: 
        """
        returns Hessian of the sum of $f$-terms
        """
    def getFeatureTypes(self) -> typing.List[ObjectiveType]: ...
    def getInitializationSample(self, previousOptima: arr = array(0.0078125)) -> arr: 
        """
        returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible
        """
    def report(self, arg0: int) -> str: 
        """
        displays semantic information on the last query
        """
    pass
class NLP_Factory(NLP):
    def __init__(self) -> None: ...
    def setBounds(self, arg0: arr, arg1: arr) -> None: ...
    def setDimension(self, arg0: int) -> None: ...
    def setEvalCallback(self, arg0: typing.Callable[[arr], typing.Tuple[arr, arr]]) -> None: ...
    @staticmethod
    def setFeatureTypes(*args, **kwargs) -> typing.Any: ...
    def testCallingEvalCallback(self, arg0: arr) -> typing.Tuple[arr, arr]: ...
    pass
class NLP_Solver():
    """
    An interface to portfolio of solvers
    """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, problem: NLP, verbose: int = 0) -> None: ...
    def getOptions(self) -> NLP_SolverOptions: ...
    def getTrace_J(self) -> arr: ...
    def getTrace_costs(self) -> arr: ...
    def getTrace_phi(self) -> arr: ...
    def getTrace_x(self) -> arr: ...
    def reportLagrangeGradients(self, arg0: StringA) -> dict: ...
    def setOptions(self, verbose: int = 1, stopTolerance: float = 0.01, stopFTolerance: float = -1.0, stopGTolerance: float = -1.0, stopEvals: int = 1000, maxStep: float = 0.2, damping: float = 1.0, stepInc: float = 1.5, stepDec: float = 0.5, wolfe: float = 0.01, muInit: float = 1.0, muInc: float = 5.0, muMax: float = 10000.0, muLBInit: float = 0.1, muLBDec: float = 0.2, maxLambda: float = -1.0) -> NLP_Solver: 
        """
        set solver options
        """
    def setProblem(self, arg0: NLP) -> NLP_Solver: ...
    def setSolver(self, arg0: NLP_SolverID) -> NLP_Solver: ...
    def setTracing(self, arg0: bool, arg1: bool, arg2: bool, arg3: bool) -> NLP_Solver: ...
    def solve(self, resampleInitialization: int = -1) -> SolverReturn: ...
    pass
class NLP_SolverID():
    """
    Members:

      gradientDescent

      rprop

      LBFGS

      newton

      augmentedLag

      squaredPenalty

      logBarrier

      singleSquaredPenalty

      NLopt

      Ipopt

      Ceres
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    Ceres: _robotic.NLP_SolverID # value = <NLP_SolverID.Ceres: 10>
    Ipopt: _robotic.NLP_SolverID # value = <NLP_SolverID.Ipopt: 9>
    LBFGS: _robotic.NLP_SolverID # value = <NLP_SolverID.LBFGS: 2>
    NLopt: _robotic.NLP_SolverID # value = <NLP_SolverID.NLopt: 8>
    __members__: dict # value = {'gradientDescent': <NLP_SolverID.gradientDescent: 0>, 'rprop': <NLP_SolverID.rprop: 1>, 'LBFGS': <NLP_SolverID.LBFGS: 2>, 'newton': <NLP_SolverID.newton: 3>, 'augmentedLag': <NLP_SolverID.augmentedLag: 4>, 'squaredPenalty': <NLP_SolverID.squaredPenalty: 5>, 'logBarrier': <NLP_SolverID.logBarrier: 6>, 'singleSquaredPenalty': <NLP_SolverID.singleSquaredPenalty: 7>, 'NLopt': <NLP_SolverID.NLopt: 8>, 'Ipopt': <NLP_SolverID.Ipopt: 9>, 'Ceres': <NLP_SolverID.Ceres: 10>}
    augmentedLag: _robotic.NLP_SolverID # value = <NLP_SolverID.augmentedLag: 4>
    gradientDescent: _robotic.NLP_SolverID # value = <NLP_SolverID.gradientDescent: 0>
    logBarrier: _robotic.NLP_SolverID # value = <NLP_SolverID.logBarrier: 6>
    newton: _robotic.NLP_SolverID # value = <NLP_SolverID.newton: 3>
    rprop: _robotic.NLP_SolverID # value = <NLP_SolverID.rprop: 1>
    singleSquaredPenalty: _robotic.NLP_SolverID # value = <NLP_SolverID.singleSquaredPenalty: 7>
    squaredPenalty: _robotic.NLP_SolverID # value = <NLP_SolverID.squaredPenalty: 5>
    pass
class NLP_SolverOptions():
    """
    solver options
    """
    def __init__(self) -> None: ...
    def dict(self) -> dict: ...
    def set_damping(self, arg0: float) -> NLP_SolverOptions: ...
    def set_maxLambda(self, arg0: float) -> NLP_SolverOptions: ...
    def set_maxStep(self, arg0: float) -> NLP_SolverOptions: ...
    def set_muInc(self, arg0: float) -> NLP_SolverOptions: ...
    def set_muInit(self, arg0: float) -> NLP_SolverOptions: ...
    def set_muLBDec(self, arg0: float) -> NLP_SolverOptions: ...
    def set_muLBInit(self, arg0: float) -> NLP_SolverOptions: ...
    def set_muMax(self, arg0: float) -> NLP_SolverOptions: ...
    def set_stepDec(self, arg0: float) -> NLP_SolverOptions: ...
    def set_stepInc(self, arg0: float) -> NLP_SolverOptions: ...
    def set_stopEvals(self, arg0: int) -> NLP_SolverOptions: ...
    def set_stopFTolerance(self, arg0: float) -> NLP_SolverOptions: ...
    def set_stopGTolerance(self, arg0: float) -> NLP_SolverOptions: ...
    def set_stopTolerance(self, arg0: float) -> NLP_SolverOptions: ...
    def set_verbose(self, arg0: int) -> NLP_SolverOptions: ...
    def set_wolfe(self, arg0: float) -> NLP_SolverOptions: ...
    pass
class OT():
    """
    Members:

      none

      f

      sos

      ineq

      eq

      ineqB

      ineqP
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'none': <OT.none: 0>, 'f': <OT.f: 1>, 'sos': <OT.sos: 2>, 'ineq': <OT.ineq: 3>, 'eq': <OT.eq: 4>, 'ineqB': <OT.ineqB: 5>, 'ineqP': <OT.ineqP: 6>}
    eq: _robotic.OT # value = <OT.eq: 4>
    f: _robotic.OT # value = <OT.f: 1>
    ineq: _robotic.OT # value = <OT.ineq: 3>
    ineqB: _robotic.OT # value = <OT.ineqB: 5>
    ineqP: _robotic.OT # value = <OT.ineqP: 6>
    none: _robotic.OT # value = <OT.none: 0>
    sos: _robotic.OT # value = <OT.sos: 2>
    pass
class OptBench_Skeleton_Handover():
    def __init__(self, arg0: ArgWord) -> None: ...
    def get(self) -> NLP: ...
    pass
class OptBench_Skeleton_Pick():
    def __init__(self, arg0: ArgWord) -> None: ...
    def get(self) -> NLP: ...
    pass
class OptBench_Skeleton_StackAndBalance():
    def __init__(self, arg0: ArgWord) -> None: ...
    def get(self) -> NLP: ...
    pass
class OptBenchmark_InvKin_Endeff():
    def __init__(self, arg0: str, arg1: bool) -> None: ...
    def get(self) -> NLP: ...
    pass
class PathFinder():
    """
    todo doc
    """
    def __init__(self) -> None: ...
    def setExplicitCollisionPairs(self, collisionPairs: StringA) -> PathFinder: ...
    def setProblem(self, Configuration: Config, starts: arr, goals: arr) -> PathFinder: ...
    def solve(self) -> SolverReturn: ...
    def step(self) -> bool: ...
    pass
class PointCloudViewer():
    pass
class ST():
    """
    Members:

      none

      box

      sphere

      capsule

      mesh

      cylinder

      marker

      pointCloud

      ssCvx

      ssBox

      ssCylinder

      ssBoxElip

      quad

      camera

      sdf
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'none': <ST.none: -1>, 'box': <ST.box: 0>, 'sphere': <ST.sphere: 1>, 'capsule': <ST.capsule: 2>, 'mesh': <ST.mesh: 3>, 'cylinder': <ST.cylinder: 4>, 'marker': <ST.marker: 5>, 'pointCloud': <ST.pointCloud: 6>, 'ssCvx': <ST.ssCvx: 7>, 'ssBox': <ST.ssBox: 8>, 'ssCylinder': <ST.ssCylinder: 9>, 'ssBoxElip': <ST.ssBoxElip: 10>, 'quad': <ST.quad: 11>, 'camera': <ST.camera: 12>, 'sdf': <ST.sdf: 13>}
    box: _robotic.ST # value = <ST.box: 0>
    camera: _robotic.ST # value = <ST.camera: 12>
    capsule: _robotic.ST # value = <ST.capsule: 2>
    cylinder: _robotic.ST # value = <ST.cylinder: 4>
    marker: _robotic.ST # value = <ST.marker: 5>
    mesh: _robotic.ST # value = <ST.mesh: 3>
    none: _robotic.ST # value = <ST.none: -1>
    pointCloud: _robotic.ST # value = <ST.pointCloud: 6>
    quad: _robotic.ST # value = <ST.quad: 11>
    sdf: _robotic.ST # value = <ST.sdf: 13>
    sphere: _robotic.ST # value = <ST.sphere: 1>
    ssBox: _robotic.ST # value = <ST.ssBox: 8>
    ssBoxElip: _robotic.ST # value = <ST.ssBoxElip: 10>
    ssCvx: _robotic.ST # value = <ST.ssCvx: 7>
    ssCylinder: _robotic.ST # value = <ST.ssCylinder: 9>
    pass
class SY():
    """
    Members:

      touch

      above

      inside

      oppose

      restingOn

      poseEq

      positionEq

      stableRelPose

      stablePose

      stable

      stableOn

      dynamic

      dynamicOn

      dynamicTrans

      quasiStatic

      quasiStaticOn

      downUp

      break

      stableZero

      contact

      contactStick

      contactComplementary

      bounce

      push

      magic

      magicTrans

      pushAndPlace

      topBoxGrasp

      topBoxPlace

      dampMotion

      identical

      alignByInt

      makeFree

      forceBalance

      relPosY

      touchBoxNormalX

      touchBoxNormalY

      touchBoxNormalZ

      boxGraspX

      boxGraspY

      boxGraspZ

      lift

      stableYPhi

      stableOnX

      stableOnY

      end
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'touch': <SY.touch: 0>, 'above': <SY.above: 1>, 'inside': <SY.inside: 2>, 'oppose': <SY.oppose: 3>, 'restingOn': <SY.restingOn: 4>, 'poseEq': <SY.poseEq: 5>, 'positionEq': <SY.positionEq: 6>, 'stableRelPose': <SY.stableRelPose: 7>, 'stablePose': <SY.stablePose: 8>, 'stable': <SY.stable: 9>, 'stableOn': <SY.stableOn: 10>, 'dynamic': <SY.dynamic: 11>, 'dynamicOn': <SY.dynamicOn: 12>, 'dynamicTrans': <SY.dynamicTrans: 13>, 'quasiStatic': <SY.quasiStatic: 14>, 'quasiStaticOn': <SY.quasiStaticOn: 15>, 'downUp': <SY.downUp: 16>, 'break': <SY.break: 17>, 'stableZero': <SY.stableZero: 18>, 'contact': <SY.contact: 19>, 'contactStick': <SY.contactStick: 20>, 'contactComplementary': <SY.contactComplementary: 21>, 'bounce': <SY.bounce: 22>, 'push': <SY.push: 23>, 'magic': <SY.magic: 24>, 'magicTrans': <SY.magicTrans: 25>, 'pushAndPlace': <SY.pushAndPlace: 26>, 'topBoxGrasp': <SY.topBoxGrasp: 27>, 'topBoxPlace': <SY.topBoxPlace: 28>, 'dampMotion': <SY.dampMotion: 29>, 'identical': <SY.identical: 30>, 'alignByInt': <SY.alignByInt: 31>, 'makeFree': <SY.makeFree: 32>, 'forceBalance': <SY.forceBalance: 33>, 'relPosY': <SY.relPosY: 34>, 'touchBoxNormalX': <SY.touchBoxNormalX: 35>, 'touchBoxNormalY': <SY.touchBoxNormalY: 36>, 'touchBoxNormalZ': <SY.touchBoxNormalZ: 37>, 'boxGraspX': <SY.boxGraspX: 38>, 'boxGraspY': <SY.boxGraspY: 39>, 'boxGraspZ': <SY.boxGraspZ: 40>, 'lift': <SY.lift: 41>, 'stableYPhi': <SY.stableYPhi: 42>, 'stableOnX': <SY.stableOnX: 43>, 'stableOnY': <SY.stableOnY: 44>, 'end': <SY.end: 46>}
    above: _robotic.SY # value = <SY.above: 1>
    alignByInt: _robotic.SY # value = <SY.alignByInt: 31>
    bounce: _robotic.SY # value = <SY.bounce: 22>
    boxGraspX: _robotic.SY # value = <SY.boxGraspX: 38>
    boxGraspY: _robotic.SY # value = <SY.boxGraspY: 39>
    boxGraspZ: _robotic.SY # value = <SY.boxGraspZ: 40>
    break: _robotic.SY # value = <SY.break: 17>
    contact: _robotic.SY # value = <SY.contact: 19>
    contactComplementary: _robotic.SY # value = <SY.contactComplementary: 21>
    contactStick: _robotic.SY # value = <SY.contactStick: 20>
    dampMotion: _robotic.SY # value = <SY.dampMotion: 29>
    downUp: _robotic.SY # value = <SY.downUp: 16>
    dynamic: _robotic.SY # value = <SY.dynamic: 11>
    dynamicOn: _robotic.SY # value = <SY.dynamicOn: 12>
    dynamicTrans: _robotic.SY # value = <SY.dynamicTrans: 13>
    end: _robotic.SY # value = <SY.end: 46>
    forceBalance: _robotic.SY # value = <SY.forceBalance: 33>
    identical: _robotic.SY # value = <SY.identical: 30>
    inside: _robotic.SY # value = <SY.inside: 2>
    lift: _robotic.SY # value = <SY.lift: 41>
    magic: _robotic.SY # value = <SY.magic: 24>
    magicTrans: _robotic.SY # value = <SY.magicTrans: 25>
    makeFree: _robotic.SY # value = <SY.makeFree: 32>
    oppose: _robotic.SY # value = <SY.oppose: 3>
    poseEq: _robotic.SY # value = <SY.poseEq: 5>
    positionEq: _robotic.SY # value = <SY.positionEq: 6>
    push: _robotic.SY # value = <SY.push: 23>
    pushAndPlace: _robotic.SY # value = <SY.pushAndPlace: 26>
    quasiStatic: _robotic.SY # value = <SY.quasiStatic: 14>
    quasiStaticOn: _robotic.SY # value = <SY.quasiStaticOn: 15>
    relPosY: _robotic.SY # value = <SY.relPosY: 34>
    restingOn: _robotic.SY # value = <SY.restingOn: 4>
    stable: _robotic.SY # value = <SY.stable: 9>
    stableOn: _robotic.SY # value = <SY.stableOn: 10>
    stableOnX: _robotic.SY # value = <SY.stableOnX: 43>
    stableOnY: _robotic.SY # value = <SY.stableOnY: 44>
    stablePose: _robotic.SY # value = <SY.stablePose: 8>
    stableRelPose: _robotic.SY # value = <SY.stableRelPose: 7>
    stableYPhi: _robotic.SY # value = <SY.stableYPhi: 42>
    stableZero: _robotic.SY # value = <SY.stableZero: 18>
    topBoxGrasp: _robotic.SY # value = <SY.topBoxGrasp: 27>
    topBoxPlace: _robotic.SY # value = <SY.topBoxPlace: 28>
    touch: _robotic.SY # value = <SY.touch: 0>
    touchBoxNormalX: _robotic.SY # value = <SY.touchBoxNormalX: 35>
    touchBoxNormalY: _robotic.SY # value = <SY.touchBoxNormalY: 36>
    touchBoxNormalZ: _robotic.SY # value = <SY.touchBoxNormalZ: 37>
    pass
class Simulation():
    """
    A direct simulation interface to physics engines (Nvidia PhysX, Bullet) -- see https://marctoussaint.github.io/robotics-course/tutorials/simulation.html
    """
    def __init__(self, C: Config, engine: SimulationEngine, verbose: int = 2) -> None: 
        """
        create a Simulation that is associated/attached to the given configuration
        """
    def addImp(self, arg0: ImpType, arg1: StringA, arg2: arr) -> None: ...
    @staticmethod
    def addSensor(*args, **kwargs) -> typing.Any: ...
    def attach(self, gripper: Frame, obj: Frame) -> None: ...
    def closeGripper(self, gripperFrameName: str, width: float = 0.05, speed: float = 0.3, force: float = 20.0) -> None: ...
    def depthData2pointCloud(self, arg0: numpy.ndarray[numpy.float32], arg1: typing.List[float]) -> numpy.ndarray[numpy.float64]: ...
    def detach(self, obj: Frame) -> None: ...
    def getGripperIsGrasping(self, gripperFrameName: str) -> bool: ...
    def getGripperWidth(self, gripperFrameName: str) -> float: ...
    def getGroundTruthPosition(self, arg0: str) -> numpy.ndarray[numpy.float64]: ...
    def getGroundTruthRotationMatrix(self, arg0: str) -> numpy.ndarray[numpy.float64]: ...
    def getGroundTruthSize(self, arg0: str) -> numpy.ndarray[numpy.float64]: ...
    def getImageAndDepth(self) -> tuple: ...
    @staticmethod
    def getScreenshot(*args, **kwargs) -> typing.Any: ...
    def getState(self) -> tuple: 
        """
        returns a 4-tuple or frame state, joint state, frame velocities (linear & angular), joint velocities
        """
    def getTimeToSplineEnd(self) -> float: ...
    def get_q(self) -> arr: ...
    def get_qDot(self) -> arr: ...
    def loadTeleopCallbacks(self) -> None: ...
    def openGripper(self, gripperFrameName: str, width: float = 0.075, speed: float = 0.3) -> None: ...
    def pushConfigurationToSimulator(self, frameVelocities: arr = array(0.0078125), jointVelocities: arr = array(0.0078125)) -> None: 
        """
        set the simulator to the full (frame) state of the configuration
        """
    def resetSplineRef(self) -> None: 
        """
        reset the spline reference, i.e., clear the current spline buffer and initialize it to constant spline at current position (to which setSplineRef can append)
        """
    @staticmethod
    def selectSensor(*args, **kwargs) -> typing.Any: ...
    def setSplineRef(self, path: arr, times: arr, append: bool = True) -> None: 
        """
        set the spline reference to generate motion
        * path: single configuration, or sequence of spline control points
        * times: array with single total duration, or time for each control point (times.N==path.d0)
        * append: append (with zero-velocity at append), or smoothly overwrite
        """
    def setState(self, frameState: arr, jointState: arr = array(0.0078125), frameVelocities: arr = array(0.0078125), jointVelocities: arr = array(0.0078125)) -> None: ...
    def step(self, u_control: arr, tau: float = 0.01, u_mode: ControlMode = ControlMode.velocity) -> None: ...
    pass
class SimulationEngine():
    """
    Members:

      physx

      bullet

      kinematic
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'physx': <SimulationEngine.physx: 1>, 'bullet': <SimulationEngine.bullet: 2>, 'kinematic': <SimulationEngine.kinematic: 3>}
    bullet: _robotic.SimulationEngine # value = <SimulationEngine.bullet: 2>
    kinematic: _robotic.SimulationEngine # value = <SimulationEngine.kinematic: 3>
    physx: _robotic.SimulationEngine # value = <SimulationEngine.physx: 1>
    pass
class Skeleton():
    def __init__(self) -> None: ...
    def add(self, arg0: list) -> None: ...
    def addEntry(self, timeInterval: arr, symbol: SY, frames: StringA) -> None: ...
    def addExplicitCollisions(self, collisions: StringA) -> None: ...
    def addLiftPriors(self, lift: StringA) -> None: ...
    def enableAccumulatedCollisions(self, enable: bool = True) -> None: ...
    def getKomo_finalSlice(self, Configuration: Config, lenScale: float, homingScale: float, collScale: float) -> KOMO: ...
    def getKomo_path(self, Configuration: Config, stepsPerPhase: int, accScale: float, lenScale: float, homingScale: float, collScale: float) -> KOMO: ...
    def getKomo_waypoints(self, Configuration: Config, lenScale: float, homingScale: float, collScale: float) -> KOMO: ...
    def getMaxPhase(self) -> float: ...
    def getTwoWaypointProblem(self, t2: int, komoWays: KOMO) -> tuple: ...
    pass
class SolverReturn():
    """
    return of nlp solve call
    """
    def __init__(self) -> None: ...
    def __str__(self) -> str: ...
    def dict(self) -> dict: ...
    @property
    def done(self) -> bool:
        """
        :type: bool
        """
    @done.setter
    def done(self, arg0: bool) -> None:
        pass
    @property
    def eq(self) -> float:
        """
        :type: float
        """
    @eq.setter
    def eq(self, arg0: float) -> None:
        pass
    @property
    def evals(self) -> int:
        """
        :type: int
        """
    @evals.setter
    def evals(self, arg0: int) -> None:
        pass
    @property
    def f(self) -> float:
        """
        :type: float
        """
    @f.setter
    def f(self, arg0: float) -> None:
        pass
    @property
    def feasible(self) -> bool:
        """
        :type: bool
        """
    @feasible.setter
    def feasible(self, arg0: bool) -> None:
        pass
    @property
    def ineq(self) -> float:
        """
        :type: float
        """
    @ineq.setter
    def ineq(self, arg0: float) -> None:
        pass
    @property
    def sos(self) -> float:
        """
        :type: float
        """
    @sos.setter
    def sos(self, arg0: float) -> None:
        pass
    @property
    def time(self) -> float:
        """
        :type: float
        """
    @time.setter
    def time(self, arg0: float) -> None:
        pass
    @property
    def x(self) -> arr:
        """
        :type: arr
        """
    @x.setter
    def x(self, arg0: arr) -> None:
        pass
    pass
def compiled() -> str:
    """
    return a compile date+time version string
    """
def depthImage2PointCloud(depth: numpy.ndarray[numpy.float32], fxycxy: arr) -> arr:
    """
    return the point cloud from the depth image
    """
def getStartGoalPath(arg0: Config, arg1: arr, arg2: arr) -> arr:
    pass
def params_add(arg0: dict) -> None:
    """
    add/set parameters
    """
def params_clear() -> None:
    """
    clear all parameters
    """
def params_file(arg0: str) -> None:
    """
    add parameters from a file
    """
def params_print() -> None:
    """
    print the parameters
    """
def raiPath(*args, **kwargs) -> typing.Any:
    """
    get a path relative to rai base path
    """
def setRaiPath(arg0: str) -> None:
    """
    redefine the rai (or rai-robotModels) path
    """
Ceres: _robotic.NLP_SolverID # value = <NLP_SolverID.Ceres: 10>
Ipopt: _robotic.NLP_SolverID # value = <NLP_SolverID.Ipopt: 9>
LBFGS: _robotic.NLP_SolverID # value = <NLP_SolverID.LBFGS: 2>
NLopt: _robotic.NLP_SolverID # value = <NLP_SolverID.NLopt: 8>
XBall: _robotic.JT # value = <JT.XBall: 15>
_left: _robotic.ArgWord # value = <ArgWord._left: 0>
_path: _robotic.ArgWord # value = <ArgWord._path: 3>
_right: _robotic.ArgWord # value = <ArgWord._right: 1>
_sequence: _robotic.ArgWord # value = <ArgWord._sequence: 2>
above: _robotic.SY # value = <SY.above: 1>
aboveBox: _robotic.FS # value = <FS.aboveBox: 33>
acceleration: _robotic.ControlMode # value = <ControlMode.acceleration: 3>
accumulatedCollisions: _robotic.FS # value = <FS.accumulatedCollisions: 27>
adversarialDropper: _robotic.ImpType # value = <ImpType.adversarialDropper: 4>
alignByInt: _robotic.SY # value = <SY.alignByInt: 31>
angularVel: _robotic.FS # value = <FS.angularVel: 26>
augmentedLag: _robotic.NLP_SolverID # value = <NLP_SolverID.augmentedLag: 4>
bounce: _robotic.SY # value = <SY.bounce: 22>
box: _robotic.ST # value = <ST.box: 0>
boxGraspX: _robotic.SY # value = <SY.boxGraspX: 38>
boxGraspY: _robotic.SY # value = <SY.boxGraspY: 39>
boxGraspZ: _robotic.SY # value = <SY.boxGraspZ: 40>
break: _robotic.SY # value = <SY.break: 17>
bullet: _robotic.SimulationEngine # value = <SimulationEngine.bullet: 2>
camera: _robotic.ST # value = <ST.camera: 12>
capsule: _robotic.ST # value = <ST.capsule: 2>
closeGripper: _robotic.ImpType # value = <ImpType.closeGripper: 0>
contact: _robotic.SY # value = <SY.contact: 19>
contactComplementary: _robotic.SY # value = <SY.contactComplementary: 21>
contactConstraints: _robotic.FS # value = <FS.contactConstraints: 42>
contactStick: _robotic.SY # value = <SY.contactStick: 20>
cylinder: _robotic.ST # value = <ST.cylinder: 4>
dampMotion: _robotic.SY # value = <SY.dampMotion: 29>
depthNoise: _robotic.ImpType # value = <ImpType.depthNoise: 2>
distance: _robotic.FS # value = <FS.distance: 29>
downUp: _robotic.SY # value = <SY.downUp: 16>
dynamic: _robotic.SY # value = <SY.dynamic: 11>
dynamicOn: _robotic.SY # value = <SY.dynamicOn: 12>
dynamicTrans: _robotic.SY # value = <SY.dynamicTrans: 13>
end: _robotic.SY # value = <SY.end: 46>
energy: _robotic.FS # value = <FS.energy: 43>
eq: _robotic.OT # value = <OT.eq: 4>
f: _robotic.OT # value = <OT.f: 1>
forceBalance: _robotic.SY # value = <SY.forceBalance: 33>
free: _robotic.JT # value = <JT.free: 16>
gazeAt: _robotic.FS # value = <FS.gazeAt: 25>
generic: _robotic.JT # value = <JT.generic: 17>
gradientDescent: _robotic.NLP_SolverID # value = <NLP_SolverID.gradientDescent: 0>
hingeX: _robotic.JT # value = <JT.hingeX: 1>
hingeY: _robotic.JT # value = <JT.hingeY: 2>
hingeZ: _robotic.JT # value = <JT.hingeZ: 3>
identical: _robotic.SY # value = <SY.identical: 30>
ineq: _robotic.OT # value = <OT.ineq: 3>
ineqB: _robotic.OT # value = <OT.ineqB: 5>
ineqP: _robotic.OT # value = <OT.ineqP: 6>
inside: _robotic.SY # value = <SY.inside: 2>
insideBox: _robotic.FS # value = <FS.insideBox: 34>
jointLimits: _robotic.FS # value = <FS.jointLimits: 28>
jointState: _robotic.FS # value = <FS.qItself: 31>
kinematic: _robotic.SimulationEngine # value = <SimulationEngine.kinematic: 3>
lift: _robotic.SY # value = <SY.lift: 41>
logBarrier: _robotic.NLP_SolverID # value = <NLP_SolverID.logBarrier: 6>
magic: _robotic.SY # value = <SY.magic: 24>
magicTrans: _robotic.SY # value = <SY.magicTrans: 25>
makeFree: _robotic.SY # value = <SY.makeFree: 32>
marker: _robotic.ST # value = <ST.marker: 5>
mesh: _robotic.ST # value = <ST.mesh: 3>
moveGripper: _robotic.ImpType # value = <ImpType.moveGripper: 1>
negDistance: _robotic.FS # value = <FS.distance: 29>
newton: _robotic.NLP_SolverID # value = <NLP_SolverID.newton: 3>
noPenetrations: _robotic.ImpType # value = <ImpType.noPenetrations: 7>
none: _robotic.OT # value = <OT.none: 0>
objectImpulses: _robotic.ImpType # value = <ImpType.objectImpulses: 5>
oppose: _robotic.SY # value = <SY.oppose: 3>
pairCollision_negScalar: _robotic.FS # value = <FS.pairCollision_negScalar: 35>
pairCollision_normal: _robotic.FS # value = <FS.pairCollision_normal: 37>
pairCollision_p1: _robotic.FS # value = <FS.pairCollision_p1: 38>
pairCollision_p2: _robotic.FS # value = <FS.pairCollision_p2: 39>
pairCollision_vector: _robotic.FS # value = <FS.pairCollision_vector: 36>
phiTransXY: _robotic.JT # value = <JT.phiTransXY: 14>
physics: _robotic.FS # value = <FS.physics: 41>
physx: _robotic.SimulationEngine # value = <SimulationEngine.physx: 1>
pointCloud: _robotic.ST # value = <ST.pointCloud: 6>
pose: _robotic.FS # value = <FS.pose: 6>
poseDiff: _robotic.FS # value = <FS.poseDiff: 7>
poseEq: _robotic.SY # value = <SY.poseEq: 5>
poseRel: _robotic.FS # value = <FS.poseRel: 8>
position: _robotic.ControlMode # value = <ControlMode.position: 1>
positionDiff: _robotic.FS # value = <FS.positionDiff: 1>
positionEq: _robotic.SY # value = <SY.positionEq: 6>
positionRel: _robotic.FS # value = <FS.positionRel: 2>
push: _robotic.SY # value = <SY.push: 23>
pushAndPlace: _robotic.SY # value = <SY.pushAndPlace: 26>
qItself: _robotic.FS # value = <FS.qItself: 31>
quad: _robotic.ST # value = <ST.quad: 11>
quasiStatic: _robotic.SY # value = <SY.quasiStatic: 14>
quasiStaticOn: _robotic.SY # value = <SY.quasiStaticOn: 15>
quatBall: _robotic.JT # value = <JT.quatBall: 13>
quaternion: _robotic.FS # value = <FS.quaternion: 3>
quaternionDiff: _robotic.FS # value = <FS.quaternionDiff: 4>
quaternionRel: _robotic.FS # value = <FS.quaternionRel: 5>
relPosY: _robotic.SY # value = <SY.relPosY: 34>
restingOn: _robotic.SY # value = <SY.restingOn: 4>
rgbNoise: _robotic.ImpType # value = <ImpType.rgbNoise: 3>
rigid: _robotic.JT # value = <JT.rigid: 12>
rprop: _robotic.NLP_SolverID # value = <NLP_SolverID.rprop: 1>
scalarProductXX: _robotic.FS # value = <FS.scalarProductXX: 18>
scalarProductXY: _robotic.FS # value = <FS.scalarProductXY: 19>
scalarProductXZ: _robotic.FS # value = <FS.scalarProductXZ: 20>
scalarProductYX: _robotic.FS # value = <FS.scalarProductYX: 21>
scalarProductYY: _robotic.FS # value = <FS.scalarProductYY: 22>
scalarProductYZ: _robotic.FS # value = <FS.scalarProductYZ: 23>
scalarProductZZ: _robotic.FS # value = <FS.scalarProductZZ: 24>
sdf: _robotic.ST # value = <ST.sdf: 13>
singleSquaredPenalty: _robotic.NLP_SolverID # value = <NLP_SolverID.singleSquaredPenalty: 7>
sos: _robotic.OT # value = <OT.sos: 2>
sphere: _robotic.ST # value = <ST.sphere: 1>
spline: _robotic.ControlMode # value = <ControlMode.spline: 5>
squaredPenalty: _robotic.NLP_SolverID # value = <NLP_SolverID.squaredPenalty: 5>
ssBox: _robotic.ST # value = <ST.ssBox: 8>
ssBoxElip: _robotic.ST # value = <ST.ssBoxElip: 10>
ssCvx: _robotic.ST # value = <ST.ssCvx: 7>
ssCylinder: _robotic.ST # value = <ST.ssCylinder: 9>
stable: _robotic.SY # value = <SY.stable: 9>
stableOn: _robotic.SY # value = <SY.stableOn: 10>
stableOnX: _robotic.SY # value = <SY.stableOnX: 43>
stableOnY: _robotic.SY # value = <SY.stableOnY: 44>
stablePose: _robotic.SY # value = <SY.stablePose: 8>
stableRelPose: _robotic.SY # value = <SY.stableRelPose: 7>
stableYPhi: _robotic.SY # value = <SY.stableYPhi: 42>
stableZero: _robotic.SY # value = <SY.stableZero: 18>
standingAbove: _robotic.FS # value = <FS.standingAbove: 40>
tau: _robotic.JT # value = <JT.tau: 18>
topBoxGrasp: _robotic.SY # value = <SY.topBoxGrasp: 27>
topBoxPlace: _robotic.SY # value = <SY.topBoxPlace: 28>
touch: _robotic.SY # value = <SY.touch: 0>
touchBoxNormalX: _robotic.SY # value = <SY.touchBoxNormalX: 35>
touchBoxNormalY: _robotic.SY # value = <SY.touchBoxNormalY: 36>
touchBoxNormalZ: _robotic.SY # value = <SY.touchBoxNormalZ: 37>
trans3: _robotic.JT # value = <JT.trans3: 8>
transAccelerations: _robotic.FS # value = <FS.transAccelerations: 44>
transVelocities: _robotic.FS # value = <FS.transVelocities: 45>
transX: _robotic.JT # value = <JT.transX: 4>
transXY: _robotic.JT # value = <JT.transXY: 7>
transXYPhi: _robotic.JT # value = <JT.transXYPhi: 9>
transY: _robotic.JT # value = <JT.transY: 5>
transYPhi: _robotic.JT # value = <JT.transYPhi: 10>
transZ: _robotic.JT # value = <JT.transZ: 6>
universal: _robotic.JT # value = <JT.universal: 11>
vectorX: _robotic.FS # value = <FS.vectorX: 9>
vectorXDiff: _robotic.FS # value = <FS.vectorXDiff: 10>
vectorXRel: _robotic.FS # value = <FS.vectorXRel: 11>
vectorY: _robotic.FS # value = <FS.vectorY: 12>
vectorYDiff: _robotic.FS # value = <FS.vectorYDiff: 13>
vectorYRel: _robotic.FS # value = <FS.vectorYRel: 14>
vectorZ: _robotic.FS # value = <FS.vectorZ: 15>
vectorZDiff: _robotic.FS # value = <FS.vectorZDiff: 16>
vectorZRel: _robotic.FS # value = <FS.vectorZRel: 17>
velocity: _robotic.ControlMode # value = <ControlMode.velocity: 2>
