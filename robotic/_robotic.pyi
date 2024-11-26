"""
rai bindings
"""
from __future__ import annotations
import numpy
import typing
from . import DataGen
from . import test
__all__ = ['ArgWord', 'BotOp', 'CameraView', 'CameraViewSensor', 'Config', 'ConfigurationViewer', 'ControlMode', 'DataGen', 'FS', 'Frame', 'JT', 'KOMO', 'KOMO_Objective', 'LGP_Tool', 'Logic2KOMO_Translator', 'NLP', 'NLP_Factory', 'NLP_Sampler', 'NLP_Solver', 'NLP_SolverID', 'NLP_SolverOptions', 'OT', 'OptBench_Skeleton_Handover', 'OptBench_Skeleton_Pick', 'OptBench_Skeleton_StackAndBalance', 'OptBenchmark_InvKin_Endeff', 'PathFinder', 'ST', 'SY', 'Simulation', 'SimulationEngine', 'Skeleton', 'SolverReturn', 'TAMP_Provider', 'compiled', 'default_Logic2KOMO_Translator', 'default_TAMP_Provider', 'depthImage2PointCloud', 'params_add', 'params_clear', 'params_file', 'params_print', 'raiPath', 'setRaiPath', 'test']
class ArgWord:
    """
    [todo: replace by str]
    
    Members:
    
      _left
    
      _right
    
      _sequence
    
      _path
    """
    __members__: typing.ClassVar[dict[str, ArgWord]]  # value = {'_left': <ArgWord._left: 0>, '_right': <ArgWord._right: 1>, '_sequence': <ArgWord._sequence: 2>, '_path': <ArgWord._path: 3>}
    _left: typing.ClassVar[ArgWord]  # value = <ArgWord._left: 0>
    _path: typing.ClassVar[ArgWord]  # value = <ArgWord._path: 3>
    _right: typing.ClassVar[ArgWord]  # value = <ArgWord._right: 1>
    _sequence: typing.ClassVar[ArgWord]  # value = <ArgWord._sequence: 2>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class BotOp:
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
    def getGripperPos(self, leftRight: ArgWord) -> float:
        """
        returns the gripper pos
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
        close gripper and indicate what should be grasped -- makes no difference in real, but helps simulation to mimic grasping more reliably
        """
    def gripperDone(self, leftRight: ArgWord) -> bool:
        """
        returns if gripper is done
        """
    def gripperMove(self, leftRight: ArgWord, width: float = 0.075, speed: float = 0.2) -> None:
        """
        move the gripper to width (default: open)
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
    def sync(self, C: Config, waitTime: float = 0.1, viewMsg: ... = '') -> int:
        """
        sync your workspace configuration C with the robot state
        """
    def wait(self, C: Config, forKeyPressed: bool = True, forTimeToEnd: bool = True, forGripper: bool = False) -> int:
        """
        repeatedly sync your workspace C until a key is pressed or motion ends (optionally)
        """
class CameraView:
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
    def computeSegmentationID(self) -> ...:
        """
        return a uint16 array with object ID segmentation
        """
    def computeSegmentationImage(self) -> ...:
        """
        return an rgb image encoding the object ID segmentation
        """
    def getFxycxy(self) -> arr:
        """
        return the camera intrinsics f_x, f_y, c_x, c_y
        """
    def setCamera(self, cameraFrameName: Frame) -> ...:
        """
        select a camera, typically a frame that has camera info attributes
        """
class CameraViewSensor:
    pass
class Config:
    """
    Core data structure to represent a kinematic configuration (essentially a tree of frames). See https://marctoussaint.github.io/robotics-course/tutorials/1a-configurations.html
    """
    def __init__(self) -> None:
        """
        initializes to an empty configuration, with no frames
        """
    def addConfigurationCopy(self, config: Config, prefix: ... = '', tau: float = 1.0) -> Frame:
        ...
    def addFile(self, filename: str, namePrefix: str = None) -> Frame:
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
    def animateSpline(self, T: int = 3) -> None:
        """
        animate with random spline in limits bounding box [T=#spline points]
        """
    def attach(self, arg0: str, arg1: str) -> None:
        """
        change the configuration by creating a rigid joint from frame1 to frame2, adopting their current relative pose. This also breaks the first joint that is parental to frame2 and reverses the topological order from frame2 to the broken joint
        """
    def checkConsistency(self) -> bool:
        """
        internal use
        """
    def clear(self) -> None:
        """
        clear all frames and additional data; becomes the empty configuration, with no frames
        """
    def computeCollisions(self) -> None:
        """
        [should be obsolete; getCollision* methods auto ensure proxies] call the broadphase collision engine (SWIFT++ or FCL) to generate the list of collisions (or near proximities) between all frame shapes that have the collision tag set non-zero
        """
    def delFrame(self, frameName: str) -> None:
        """
        destroy and remove a frame from C
        """
    def eval(self, featureSymbol: FS, frames: StringA = [], scale: arr = ..., target: arr = ..., order: int = -1) -> tuple:
        """
        evaluate a feature -- see https://marctoussaint.github.io/robotics-course/tutorials/features.html
        """
    def frame(self, frameName: str, warnIfNotExist: bool = True) -> Frame:
        """
        get access to a frame by name; use the Frame methods to set/get frame properties
        """
    def getCollidablePairs(self) -> StringA:
        """
        returns the list of collisable pairs -- this should help debugging the 'contact' flag settings in a configuration
        """
    def getCollisionFree(self) -> bool:
        """
        returns if the configuration is collision free (binary collision check, using FCL only; collidable objects need to have contact flag)
        """
    def getCollisions(self, belowMargin: float = 1.0) -> list:
        """
        return the results of collision computations: a list of 3 tuples with (frame1, frame2, distance). Optionally report only on distances below a margin To get really precise distances and penetrations use the FS.distance feature with the two frame names
        """
    def getCollisionsTotalPenetration(self) -> float:
        """
        returns the sum of all penetrations (using FCL for broadphase; and low-level GJK/MRP for fine pair-wise distance/penetration computation)
        """
    def getFrame(self, frameName: str, warnIfNotExist: bool = True) -> Frame:
        """
        get access to a frame by name; use the Frame methods to set/get frame properties
        """
    def getFrameDimension(self) -> int:
        """
        get the total number of frames
        """
    def getFrameNames(self) -> list[str]:
        """
        get the list of frame names
        """
    def getFrameState(self) -> numpy.ndarray[numpy.float64]:
        """
        get the frame state as a n-times-7 numpy matrix, with a 7D pose per frame
        """
    def getFrames(self) -> list[Frame]:
        ...
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
    def get_viewer(self, window_title: str = None, offscreen: bool = False) -> ...:
        ...
    def report(self) -> str:
        """
        return a string with basic info (#frames, etc)
        """
    def selectJoints(self, jointNames: list[str], notThose: bool = False) -> None:
        """
        redefine what are considered the DOFs of this configuration: only joints listed in jointNames are considered part of the joint state and define the number of DOFs
        """
    def selectJointsBySubtree(self, root: Frame) -> None:
        ...
    @typing.overload
    def setFrameState(self, X: list[float], frames: list[str] = []) -> None:
        """
        set the frame state, optionally only for a subset of frames specified as list of frame names
        """
    @typing.overload
    def setFrameState(self, X: numpy.ndarray, frames: list[str] = []) -> None:
        """
        set the frame state, optionally only for a subset of frames specified as list of frame names
        """
    def setJointState(self, q: arr, joints: list = []) -> None:
        """
        set the joint state, optionally only for a subset of joints specified as list of joint names
        """
    def setJointStateSlice(self, arg0: list[float], arg1: int) -> None:
        ...
    def set_viewer(self, arg0: ...) -> None:
        ...
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
    def view_fxycxy(self) -> arr:
        """
        return (fx, fy, cx, cy): the focal length and image center in PIXEL UNITS
        """
    def view_getDepth(self) -> numpy.ndarray[numpy.float32]:
        ...
    def view_getRgb(self) -> numpy.ndarray[numpy.uint8]:
        ...
    def view_pose(self) -> arr:
        """
        return the 7D pose of the view camera
        """
    def view_raise(self) -> None:
        """
        raise the view
        """
    def view_recopyMeshes(self) -> None:
        ...
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
class ConfigurationViewer:
    """
    internal viewer handle (gl window)
    """
class ControlMode:
    """
    Members:
    
      none
    
      position
    
      velocity
    
      acceleration
    
      spline
    """
    __members__: typing.ClassVar[dict[str, ControlMode]]  # value = {'none': <ControlMode.none: 0>, 'position': <ControlMode.position: 1>, 'velocity': <ControlMode.velocity: 2>, 'acceleration': <ControlMode.acceleration: 3>, 'spline': <ControlMode.spline: 5>}
    acceleration: typing.ClassVar[ControlMode]  # value = <ControlMode.acceleration: 3>
    none: typing.ClassVar[ControlMode]  # value = <ControlMode.none: 0>
    position: typing.ClassVar[ControlMode]  # value = <ControlMode.position: 1>
    spline: typing.ClassVar[ControlMode]  # value = <ControlMode.spline: 5>
    velocity: typing.ClassVar[ControlMode]  # value = <ControlMode.velocity: 2>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class FS:
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
    __members__: typing.ClassVar[dict[str, FS]]  # value = {'position': <FS.position: 0>, 'positionDiff': <FS.positionDiff: 1>, 'positionRel': <FS.positionRel: 2>, 'quaternion': <FS.quaternion: 3>, 'quaternionDiff': <FS.quaternionDiff: 4>, 'quaternionRel': <FS.quaternionRel: 5>, 'pose': <FS.pose: 6>, 'poseDiff': <FS.poseDiff: 7>, 'poseRel': <FS.poseRel: 8>, 'vectorX': <FS.vectorX: 9>, 'vectorXDiff': <FS.vectorXDiff: 10>, 'vectorXRel': <FS.vectorXRel: 11>, 'vectorY': <FS.vectorY: 12>, 'vectorYDiff': <FS.vectorYDiff: 13>, 'vectorYRel': <FS.vectorYRel: 14>, 'vectorZ': <FS.vectorZ: 15>, 'vectorZDiff': <FS.vectorZDiff: 16>, 'vectorZRel': <FS.vectorZRel: 17>, 'scalarProductXX': <FS.scalarProductXX: 18>, 'scalarProductXY': <FS.scalarProductXY: 19>, 'scalarProductXZ': <FS.scalarProductXZ: 20>, 'scalarProductYX': <FS.scalarProductYX: 21>, 'scalarProductYY': <FS.scalarProductYY: 22>, 'scalarProductYZ': <FS.scalarProductYZ: 23>, 'scalarProductZZ': <FS.scalarProductZZ: 24>, 'gazeAt': <FS.gazeAt: 25>, 'angularVel': <FS.angularVel: 26>, 'accumulatedCollisions': <FS.accumulatedCollisions: 27>, 'jointLimits': <FS.jointLimits: 28>, 'distance': <FS.distance: 29>, 'negDistance': <FS.distance: 29>, 'oppose': <FS.oppose: 30>, 'qItself': <FS.qItself: 31>, 'jointState': <FS.qItself: 31>, 'aboveBox': <FS.aboveBox: 33>, 'insideBox': <FS.insideBox: 34>, 'pairCollision_negScalar': <FS.pairCollision_negScalar: 35>, 'pairCollision_vector': <FS.pairCollision_vector: 36>, 'pairCollision_normal': <FS.pairCollision_normal: 37>, 'pairCollision_p1': <FS.pairCollision_p1: 38>, 'pairCollision_p2': <FS.pairCollision_p2: 39>, 'standingAbove': <FS.standingAbove: 40>, 'physics': <FS.physics: 41>, 'contactConstraints': <FS.contactConstraints: 42>, 'energy': <FS.energy: 43>, 'transAccelerations': <FS.transAccelerations: 44>, 'transVelocities': <FS.transVelocities: 45>}
    aboveBox: typing.ClassVar[FS]  # value = <FS.aboveBox: 33>
    accumulatedCollisions: typing.ClassVar[FS]  # value = <FS.accumulatedCollisions: 27>
    angularVel: typing.ClassVar[FS]  # value = <FS.angularVel: 26>
    contactConstraints: typing.ClassVar[FS]  # value = <FS.contactConstraints: 42>
    distance: typing.ClassVar[FS]  # value = <FS.distance: 29>
    energy: typing.ClassVar[FS]  # value = <FS.energy: 43>
    gazeAt: typing.ClassVar[FS]  # value = <FS.gazeAt: 25>
    insideBox: typing.ClassVar[FS]  # value = <FS.insideBox: 34>
    jointLimits: typing.ClassVar[FS]  # value = <FS.jointLimits: 28>
    jointState: typing.ClassVar[FS]  # value = <FS.qItself: 31>
    negDistance: typing.ClassVar[FS]  # value = <FS.distance: 29>
    oppose: typing.ClassVar[FS]  # value = <FS.oppose: 30>
    pairCollision_negScalar: typing.ClassVar[FS]  # value = <FS.pairCollision_negScalar: 35>
    pairCollision_normal: typing.ClassVar[FS]  # value = <FS.pairCollision_normal: 37>
    pairCollision_p1: typing.ClassVar[FS]  # value = <FS.pairCollision_p1: 38>
    pairCollision_p2: typing.ClassVar[FS]  # value = <FS.pairCollision_p2: 39>
    pairCollision_vector: typing.ClassVar[FS]  # value = <FS.pairCollision_vector: 36>
    physics: typing.ClassVar[FS]  # value = <FS.physics: 41>
    pose: typing.ClassVar[FS]  # value = <FS.pose: 6>
    poseDiff: typing.ClassVar[FS]  # value = <FS.poseDiff: 7>
    poseRel: typing.ClassVar[FS]  # value = <FS.poseRel: 8>
    position: typing.ClassVar[FS]  # value = <FS.position: 0>
    positionDiff: typing.ClassVar[FS]  # value = <FS.positionDiff: 1>
    positionRel: typing.ClassVar[FS]  # value = <FS.positionRel: 2>
    qItself: typing.ClassVar[FS]  # value = <FS.qItself: 31>
    quaternion: typing.ClassVar[FS]  # value = <FS.quaternion: 3>
    quaternionDiff: typing.ClassVar[FS]  # value = <FS.quaternionDiff: 4>
    quaternionRel: typing.ClassVar[FS]  # value = <FS.quaternionRel: 5>
    scalarProductXX: typing.ClassVar[FS]  # value = <FS.scalarProductXX: 18>
    scalarProductXY: typing.ClassVar[FS]  # value = <FS.scalarProductXY: 19>
    scalarProductXZ: typing.ClassVar[FS]  # value = <FS.scalarProductXZ: 20>
    scalarProductYX: typing.ClassVar[FS]  # value = <FS.scalarProductYX: 21>
    scalarProductYY: typing.ClassVar[FS]  # value = <FS.scalarProductYY: 22>
    scalarProductYZ: typing.ClassVar[FS]  # value = <FS.scalarProductYZ: 23>
    scalarProductZZ: typing.ClassVar[FS]  # value = <FS.scalarProductZZ: 24>
    standingAbove: typing.ClassVar[FS]  # value = <FS.standingAbove: 40>
    transAccelerations: typing.ClassVar[FS]  # value = <FS.transAccelerations: 44>
    transVelocities: typing.ClassVar[FS]  # value = <FS.transVelocities: 45>
    vectorX: typing.ClassVar[FS]  # value = <FS.vectorX: 9>
    vectorXDiff: typing.ClassVar[FS]  # value = <FS.vectorXDiff: 10>
    vectorXRel: typing.ClassVar[FS]  # value = <FS.vectorXRel: 11>
    vectorY: typing.ClassVar[FS]  # value = <FS.vectorY: 12>
    vectorYDiff: typing.ClassVar[FS]  # value = <FS.vectorYDiff: 13>
    vectorYRel: typing.ClassVar[FS]  # value = <FS.vectorYRel: 14>
    vectorZ: typing.ClassVar[FS]  # value = <FS.vectorZ: 15>
    vectorZDiff: typing.ClassVar[FS]  # value = <FS.vectorZDiff: 16>
    vectorZRel: typing.ClassVar[FS]  # value = <FS.vectorZRel: 17>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Frame:
    """
    A (coordinate) frame of a configuration, which can have a parent, and associated shape, joint, and/or inertia
    """
    name: ...
    def addAttributes(self, arg0: dict) -> None:
        """
        add/set attributes for the frame
        """
    def getAttributes(self) -> dict:
        """
        get frame attributes
        """
    def getJointState(self) -> arr:
        ...
    def getMesh(self) -> tuple:
        ...
    def getMeshColors(self) -> ...:
        ...
    def getMeshPoints(self) -> arr:
        ...
    def getMeshTriangles(self) -> uintA:
        ...
    def getParent(self) -> Frame:
        ...
    def getPose(self) -> arr:
        ...
    def getPosition(self) -> arr:
        ...
    def getQuaternion(self) -> arr:
        ...
    def getRelativePosition(self) -> arr:
        ...
    def getRelativeQuaternion(self) -> arr:
        ...
    def getRelativeTransform(self) -> arr:
        ...
    def getRotationMatrix(self) -> arr:
        ...
    def getShapeType(self) -> ST:
        ...
    def getSize(self) -> arr:
        ...
    def getTransform(self) -> arr:
        ...
    def info(self) -> dict:
        ...
    def makeRoot(self, untilPartBreak: bool) -> None:
        ...
    def setAttribute(self, arg0: str, arg1: float) -> Frame:
        ...
    def setColor(self, arg0: arr) -> Frame:
        ...
    def setContact(self, arg0: int) -> Frame:
        ...
    def setConvexMesh(self, points: arr, colors: ... = ..., radius: float = 0.0) -> Frame:
        """
        attach a convex mesh as shape
        """
    def setDensity(self, data: ..., size: arr) -> Frame:
        ...
    def setImplicitSurface(self, data: ..., size: arr, blur: int, resample: float = -1.0) -> Frame:
        ...
    def setJoint(self, jointType: JT, limits: arr = ...) -> Frame:
        ...
    def setJointState(self, arg0: arr) -> Frame:
        ...
    def setMass(self, arg0: float) -> Frame:
        ...
    def setMesh(self, vertices: arr, triangles: uintA, colors: ... = ..., cvxParts: uintA = ...) -> Frame:
        """
        attach a mesh shape
        """
    def setMeshAsLines(self, arg0: list[float]) -> None:
        ...
    def setParent(self, parent: Frame, keepAbsolutePose_and_adaptRelativePose: bool = False, checkForLoop: bool = False) -> Frame:
        ...
    def setPointCloud(self, points: arr, colors: ... = ..., normals: arr = ...) -> Frame:
        """
        attach a point cloud shape
        """
    def setPose(self, arg0: str) -> None:
        ...
    def setPosition(self, arg0: arr) -> Frame:
        ...
    def setQuaternion(self, arg0: arr) -> Frame:
        ...
    def setRelativePose(self, arg0: str) -> None:
        ...
    def setRelativePosition(self, arg0: arr) -> Frame:
        ...
    def setRelativeQuaternion(self, arg0: arr) -> Frame:
        ...
    def setShape(self, type: ST, size: arr) -> Frame:
        ...
    def unLink(self) -> Frame:
        ...
class JT:
    """
    Members:
    
      none
    
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
    XBall: typing.ClassVar[JT]  # value = <JT.XBall: 15>
    __members__: typing.ClassVar[dict[str, JT]]  # value = {'none': <JT.none: 0>, 'hingeX': <JT.hingeX: 1>, 'hingeY': <JT.hingeY: 2>, 'hingeZ': <JT.hingeZ: 3>, 'transX': <JT.transX: 4>, 'transY': <JT.transY: 5>, 'transZ': <JT.transZ: 6>, 'transXY': <JT.transXY: 7>, 'trans3': <JT.trans3: 8>, 'transXYPhi': <JT.transXYPhi: 9>, 'transYPhi': <JT.transYPhi: 10>, 'universal': <JT.universal: 11>, 'rigid': <JT.rigid: 12>, 'quatBall': <JT.quatBall: 13>, 'phiTransXY': <JT.phiTransXY: 14>, 'XBall': <JT.XBall: 15>, 'free': <JT.free: 16>, 'generic': <JT.generic: 17>, 'tau': <JT.tau: 18>}
    free: typing.ClassVar[JT]  # value = <JT.free: 16>
    generic: typing.ClassVar[JT]  # value = <JT.generic: 17>
    hingeX: typing.ClassVar[JT]  # value = <JT.hingeX: 1>
    hingeY: typing.ClassVar[JT]  # value = <JT.hingeY: 2>
    hingeZ: typing.ClassVar[JT]  # value = <JT.hingeZ: 3>
    none: typing.ClassVar[JT]  # value = <JT.none: 0>
    phiTransXY: typing.ClassVar[JT]  # value = <JT.phiTransXY: 14>
    quatBall: typing.ClassVar[JT]  # value = <JT.quatBall: 13>
    rigid: typing.ClassVar[JT]  # value = <JT.rigid: 12>
    tau: typing.ClassVar[JT]  # value = <JT.tau: 18>
    trans3: typing.ClassVar[JT]  # value = <JT.trans3: 8>
    transX: typing.ClassVar[JT]  # value = <JT.transX: 4>
    transXY: typing.ClassVar[JT]  # value = <JT.transXY: 7>
    transXYPhi: typing.ClassVar[JT]  # value = <JT.transXYPhi: 9>
    transY: typing.ClassVar[JT]  # value = <JT.transY: 5>
    transYPhi: typing.ClassVar[JT]  # value = <JT.transYPhi: 10>
    transZ: typing.ClassVar[JT]  # value = <JT.transZ: 6>
    universal: typing.ClassVar[JT]  # value = <JT.universal: 11>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class KOMO:
    """
    A framework to define manipulation problems (IK, path optimization, sequential manipulation) as Nonlinear Mathematical Program (NLP). The actual NLP_Solver class is separate. (KOMO = k-order Markov Optimization) -- see https://marctoussaint.github.io/robotics-course/tutorials/1c-komo.html
    """
    @typing.overload
    def __init__(self) -> None:
        """
        [deprecated] please use the other constructor
        """
    @typing.overload
    def __init__(self, config: Config, phases: float, slicesPerPhase: int, kOrder: int, enableCollisions: bool) -> None:
        """
        constructor
        * config: the configuration, which is copied once (for IK) or many times (for waypoints/paths) to be the optimization variable
        * phases: the number P of phases (which essentially defines the real-valued interval [0,P] over which objectives can be formulated)
        * slicesPerPhase: the discretizations per phase -> in total we have phases*slicesPerPhases configurations which form the path and over which we optimize
        * kOrder: the 'Markov-order', i.e., maximal tuple of configurations over which we formulate features (e.g. take finite differences)
        * enableCollisions: if True, KOMO runs a broadphase collision check (using libFCL) in each optimization step -- only then accumulative collision/penetration features will correctly evaluate to non-zero. But this is costly.
        """
    def addControlObjective(self, times: arr, order: int, scale: float = 1.0, target: arr = ..., deltaFromSlice: int = 0, deltaToSlice: int = 0) -> Objective:
        """
        * times: (as for `addObjective`) the phase-interval in which this objective holds; [] means all times
        * order: Do we penalize the jointState directly (order=0: penalizing sqr distance to qHome, order=1: penalizing sqr distances between consecutive configurations (velocities), order=2: penalizing accelerations across 3 configurations)
        * scale: as usual, but modulated by a factor 'sqrt(delta t)' that somehow ensures total control costs in approximately independent of the choice of stepsPerPhase
        """
    def addModeSwitch(self, times: arr, newMode: SY, frames: StringA, firstSwitch: bool = True) -> None:
        ...
    def addObjective(self, times: arr, feature: FS, frames: StringA, type: ObjectiveType, scale: arr = ..., target: arr = ..., order: int = -1) -> None:
        """
        central method to define objectives in the KOMO NLP:
        * times: the time intervals (subset of configurations in a path) over which this feature is active (irrelevant for IK)
        * feature: the feature symbol (see advanced `Feature` tutorial)
        * frames: the frames for which the feature is computed, given as list of frame names
        * type: whether this is a sum-of-squares (sos) cost, or eq or ineq constraint
        * scale: the matrix(!) by which the feature is multiplied
        * target: the offset which is substracted from the feature (before scaling)
        """
    def addQuaternionNorms(self, times: arr = ..., scale: float = 3.0, hard: bool = True) -> None:
        ...
    def addRigidSwitch(self, times: float, frames: StringA, noJumpStart: bool = True) -> None:
        ...
    def addStableFrame(self, name: str, parent: str, jointType: JT, stable: bool, initName: str = None, initFrame: Frame = None) -> Frame:
        """
        complicated...
        """
    def addTimeOptimization(self) -> None:
        ...
    def clearObjectives(self) -> None:
        ...
    def getConfig(self) -> Config:
        ...
    def getFeatureNames(self) -> StringA:
        """
        (This is to be passed to the NLP_Solver when needed.) returns a long list of features (per time slice!)
        """
    def getForceInteractions(self) -> list:
        ...
    def getFrame(self, frameName: str, phaseTime: float) -> Frame:
        ...
    def getFrameState(self, arg0: int) -> arr:
        ...
    def getPath(self) -> arr:
        ...
    def getPathFrames(self) -> arr:
        ...
    def getPathTau(self) -> arr:
        ...
    def getPath_qAll(self) -> arrA:
        ...
    def getSubProblem(self, phase: int) -> tuple:
        """
        return a tuple of (configuration, start q0, end q1) for given phase of this komo problem
        """
    def getT(self) -> int:
        ...
    def get_viewer(self) -> ConfigurationViewer:
        ...
    def info_objectiveErrorTraces(self) -> arr:
        """
        return a TxO, for O objectives
        """
    def info_objectiveNames(self) -> StringA:
        """
        return a array of O strings, for O objectives
        """
    def info_sliceCollisions(self, t: int, belowMargin: float) -> ...:
        """
        return string info of collosions belowMargin in slice t
        """
    def info_sliceErrors(self, t: int, errorTraces: arr) -> ...:
        """
        return string info of objectives and errors in slice t -- needs errorTraces as input
        """
    def initOrg(self) -> None:
        ...
    def initPhaseWithDofsPath(self, t_phase: int, dofIDs: uintA, path: arr, autoResamplePath: bool = False) -> None:
        ...
    def initRandom(self, verbose: int = 0) -> None:
        ...
    def initWithConstant(self, q: arr) -> None:
        ...
    def initWithPath(self, q: arr) -> None:
        ...
    def initWithWaypoints(self, waypoints: arrA, waypointSlicesPerPhase: int = 1, interpolate: bool = False, qHomeInterpolate: float = 0.0, verbose: int = -1) -> uintA:
        ...
    def nlp(self) -> NLP:
        """
        return the problem NLP
        """
    def report(self, specs: bool = False, listObjectives: bool = True, plotOverTime: bool = False) -> ...:
        """
        returns a dict with full list of features, optionally also on problem specs and plotting costs/violations over time
        """
    def setConfig(self, config: Config, enableCollisions: bool) -> None:
        """
        [deprecated] please set directly in constructor
        """
    def setTiming(self, phases: float, slicesPerPhase: int, durationPerPhase: float, kOrder: int) -> None:
        """
        [deprecated] please set directly in constructor
        """
    def set_viewer(self, arg0: ConfigurationViewer) -> None:
        ...
    def updateRootObjects(self, config: Config) -> None:
        """
        update root frames (without parents) within all KOMO configurations
        """
    def view(self, pause: bool = False, txt: str = None) -> int:
        ...
    def view_close(self) -> None:
        ...
    def view_play(self, pause: bool = False, txt: str = None, delay: float = 0.1, saveVideoPath: str = None) -> int:
        ...
    def view_slice(self, t: int, pause: bool = False) -> int:
        ...
class KOMO_Objective:
    pass
class LGP_Tool:
    """
    Tools to compute things (and solve) a Task-and-Motion Planning problem formulated as Logic-Geometric Program
    """
    def __init__(self, arg0: Config, arg1: TAMP_Provider, arg2: Logic2KOMO_Translator) -> None:
        """
        initialization
        """
    def getSolvedKOMO(self) -> KOMO:
        """
        return the solved KOMO object (including its continuous solution) of current solution
        """
    def getSolvedPlan(self) -> StringAA:
        """
        return list of discrete decisions of current solution
        """
    def get_fullMotionProblem(self, initWithWaypoints: bool) -> KOMO:
        """
        return the (unsolved) KOMO object corresponding to the full joint motion problem spanning all steps
        """
    def get_piecewiseMotionProblem(self, phase: int, fixEnd: bool) -> KOMO:
        """
        return the (unsolved) KOMO object corresponding to the k-th piece of the current solution
        """
    def solve(self, verbose: int = 1) -> None:
        """
        compute new solution
        """
    def solveFullMotion(self, verbose: int = 1) -> KOMO:
        """
        solve full motion of current solution and return the (solved) KOMO object
        """
    def solvePiecewiseMotions(self, verbose: int = 1) -> arrA:
        """
        solve full motion of current solution and return the (solved) KOMO object
        """
    def view_close(self) -> None:
        ...
    def view_solved(self, pause: bool) -> int:
        """
        view last computed solution
        """
class Logic2KOMO_Translator:
    """
    Logic2KOMO_Translator
    """
class NLP:
    """
    Representation of a Nonlinear Mathematical Program
    """
    def checkHessian(self, x: arr, tolerance: float) -> bool:
        ...
    def checkJacobian(self, x: arr, tolerance: float, featureNames: StringA = []) -> bool:
        ...
    def evaluate(self, arg0: arr) -> tuple[arr, arr]:
        """
        query the NLP at a point $x$; returns the tuple $(phi,J)$, which is the feature vector and its Jacobian; features define cost terms, sum-of-square (sos) terms, inequalities, and equalities depending on 'getFeatureTypes'
        """
    def getBounds(self) -> arr:
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
    def getFeatureTypes(self) -> list[ObjectiveType]:
        """
        features (entries of $phi$) can be of one of (ry.OT.f, ry.OT.sos, ry.OT.ineq, ry.OT.eq), which means (cost, sum-of-square, inequality, equality). The total cost $f(x)$ is the sum of all f-terms plus sum-of-squares of sos-terms.
        """
    def getInitializationSample(self, previousOptima: arr = ...) -> arr:
        """
        returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible
        """
    def report(self, arg0: int) -> str:
        """
        displays semantic information on the last query
        """
class NLP_Factory(NLP):
    def __init__(self) -> None:
        ...
    def setBounds(self, arg0: arr, arg1: arr) -> None:
        ...
    def setDimension(self, arg0: int) -> None:
        ...
    def setEvalCallback(self, arg0: typing.Callable[[arr], tuple[arr, arr]]) -> None:
        ...
    def setFeatureTypes(self, arg0: ...) -> None:
        ...
    def testCallingEvalCallback(self, arg0: arr) -> tuple[arr, arr]:
        ...
class NLP_Sampler:
    """
    An interface to an NLP sampler
    """
    def __init__(self, problem: NLP) -> None:
        ...
    def sample(self) -> SolverReturn:
        ...
    def setOptions(self, eps: float = 0.05, useCentering: bool = True, verbose: int = 1, seedMethod: ... = 'uni', seedCandidates: int = 10, penaltyMu: float = 1.0, downhillMethod: ... = 'GN', downhillMaxSteps: int = 50, slackStepAlpha: float = 1.0, slackMaxStep: float = 0.1, slackRegLambda: float = 0.01, ineqOverstep: float = -1, downhillNoiseMethod: ... = 'none', downhillRejectMethod: ... = 'none', downhillNoiseSigma: float = 0.1, interiorMethod: ... = 'HR', interiorBurnInSteps: int = 0, interiorSampleSteps: int = 1, interiorNoiseMethod: ... = 'iso', hitRunEqMargin: float = 0.1, interiorNoiseSigma: float = 0.5, langevinTauPrime: float = -1.0) -> NLP_Sampler:
        """
        set solver options
        """
class NLP_Solver:
    """
    An interface to portfolio of solvers
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, problem: NLP, verbose: int = 0) -> None:
        ...
    def getOptions(self) -> NLP_SolverOptions:
        ...
    def getProblem(self) -> NLP:
        ...
    def getTrace_J(self) -> arr:
        ...
    def getTrace_costs(self) -> arr:
        """
        returns steps-times-3 array with rows (f+sos-costs, ineq, eq)
        """
    def getTrace_phi(self) -> arr:
        ...
    def getTrace_x(self) -> arr:
        """
        returns steps-times-n array with queries points in each row
        """
    def reportLagrangeGradients(self, featureNames: StringA = []) -> dict:
        """
        return dictionary of Lagrange gradients per objective
        """
    def setOptions(self, verbose: int = 1, stopTolerance: float = 0.01, stopFTolerance: float = -1.0, stopGTolerance: float = -1.0, stopEvals: int = 1000, stopInners: int = 1000, stopOuters: int = 1000, maxStep: float = 0.2, damping: float = 1.0, stepInc: float = 1.5, stepDec: float = 0.5, wolfe: float = 0.01, muInit: float = 1.0, muInc: float = 5.0, muMax: float = 10000.0, muLBInit: float = 0.1, muLBDec: float = 0.2, maxLambda: float = -1.0) -> NLP_Solver:
        """
        set solver options
        """
    def setProblem(self, arg0: NLP) -> NLP_Solver:
        ...
    def setPyProblem(self, arg0: typing.Any) -> None:
        ...
    def setSolver(self, arg0: NLP_SolverID) -> NLP_Solver:
        ...
    def setTracing(self, arg0: bool, arg1: bool, arg2: bool, arg3: bool) -> NLP_Solver:
        ...
    def solve(self, resampleInitialization: int = -1, verbose: int = -1) -> SolverReturn:
        ...
class NLP_SolverID:
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
    Ceres: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.Ceres: 10>
    Ipopt: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.Ipopt: 9>
    LBFGS: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.LBFGS: 2>
    NLopt: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.NLopt: 8>
    __members__: typing.ClassVar[dict[str, NLP_SolverID]]  # value = {'gradientDescent': <NLP_SolverID.gradientDescent: 0>, 'rprop': <NLP_SolverID.rprop: 1>, 'LBFGS': <NLP_SolverID.LBFGS: 2>, 'newton': <NLP_SolverID.newton: 3>, 'augmentedLag': <NLP_SolverID.augmentedLag: 4>, 'squaredPenalty': <NLP_SolverID.squaredPenalty: 5>, 'logBarrier': <NLP_SolverID.logBarrier: 6>, 'singleSquaredPenalty': <NLP_SolverID.singleSquaredPenalty: 7>, 'NLopt': <NLP_SolverID.NLopt: 8>, 'Ipopt': <NLP_SolverID.Ipopt: 9>, 'Ceres': <NLP_SolverID.Ceres: 10>}
    augmentedLag: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.augmentedLag: 4>
    gradientDescent: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.gradientDescent: 0>
    logBarrier: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.logBarrier: 6>
    newton: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.newton: 3>
    rprop: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.rprop: 1>
    singleSquaredPenalty: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.singleSquaredPenalty: 7>
    squaredPenalty: typing.ClassVar[NLP_SolverID]  # value = <NLP_SolverID.squaredPenalty: 5>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class NLP_SolverOptions:
    """
    solver options
    """
    def __init__(self) -> None:
        ...
    def dict(self) -> dict:
        ...
    def set_damping(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_maxLambda(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_maxStep(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_muInc(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_muInit(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_muLBDec(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_muLBInit(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_muMax(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_stepDec(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_stepInc(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_stopEvals(self, arg0: int) -> NLP_SolverOptions:
        ...
    def set_stopFTolerance(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_stopGTolerance(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_stopInners(self, arg0: int) -> NLP_SolverOptions:
        ...
    def set_stopOuters(self, arg0: int) -> NLP_SolverOptions:
        ...
    def set_stopTolerance(self, arg0: float) -> NLP_SolverOptions:
        ...
    def set_verbose(self, arg0: int) -> NLP_SolverOptions:
        ...
    def set_wolfe(self, arg0: float) -> NLP_SolverOptions:
        ...
class OT:
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
    __members__: typing.ClassVar[dict[str, OT]]  # value = {'none': <OT.none: 0>, 'f': <OT.f: 1>, 'sos': <OT.sos: 2>, 'ineq': <OT.ineq: 3>, 'eq': <OT.eq: 4>, 'ineqB': <OT.ineqB: 5>, 'ineqP': <OT.ineqP: 6>}
    eq: typing.ClassVar[OT]  # value = <OT.eq: 4>
    f: typing.ClassVar[OT]  # value = <OT.f: 1>
    ineq: typing.ClassVar[OT]  # value = <OT.ineq: 3>
    ineqB: typing.ClassVar[OT]  # value = <OT.ineqB: 5>
    ineqP: typing.ClassVar[OT]  # value = <OT.ineqP: 6>
    none: typing.ClassVar[OT]  # value = <OT.none: 0>
    sos: typing.ClassVar[OT]  # value = <OT.sos: 2>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OptBench_Skeleton_Handover:
    def __init__(self, arg0: ArgWord) -> None:
        ...
    def get(self) -> NLP:
        ...
class OptBench_Skeleton_Pick:
    def __init__(self, arg0: ArgWord) -> None:
        ...
    def get(self) -> NLP:
        ...
class OptBench_Skeleton_StackAndBalance:
    def __init__(self, arg0: ArgWord) -> None:
        ...
    def get(self) -> NLP:
        ...
class OptBenchmark_InvKin_Endeff:
    def __init__(self, arg0: str, arg1: bool) -> None:
        ...
    def get(self) -> NLP:
        ...
class PathFinder:
    """
    todo doc
    """
    def __init__(self) -> None:
        ...
    def get_resampledPath(self, arg0: int) -> arr:
        ...
    def setExplicitCollisionPairs(self, collisionPairs: StringA) -> None:
        """
        only after setProblem
        """
    def setProblem(self, Configuration: Config, starts: arr, goals: arr, collisionTolerance: float = 0.0001) -> None:
        ...
    def solve(self) -> SolverReturn:
        ...
class ST:
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
    __members__: typing.ClassVar[dict[str, ST]]  # value = {'none': <ST.none: -1>, 'box': <ST.box: 0>, 'sphere': <ST.sphere: 1>, 'capsule': <ST.capsule: 2>, 'mesh': <ST.mesh: 3>, 'cylinder': <ST.cylinder: 4>, 'marker': <ST.marker: 5>, 'pointCloud': <ST.pointCloud: 6>, 'ssCvx': <ST.ssCvx: 7>, 'ssBox': <ST.ssBox: 8>, 'ssCylinder': <ST.ssCylinder: 9>, 'ssBoxElip': <ST.ssBoxElip: 10>, 'quad': <ST.quad: 11>, 'camera': <ST.camera: 12>, 'sdf': <ST.sdf: 13>}
    box: typing.ClassVar[ST]  # value = <ST.box: 0>
    camera: typing.ClassVar[ST]  # value = <ST.camera: 12>
    capsule: typing.ClassVar[ST]  # value = <ST.capsule: 2>
    cylinder: typing.ClassVar[ST]  # value = <ST.cylinder: 4>
    marker: typing.ClassVar[ST]  # value = <ST.marker: 5>
    mesh: typing.ClassVar[ST]  # value = <ST.mesh: 3>
    none: typing.ClassVar[ST]  # value = <ST.none: -1>
    pointCloud: typing.ClassVar[ST]  # value = <ST.pointCloud: 6>
    quad: typing.ClassVar[ST]  # value = <ST.quad: 11>
    sdf: typing.ClassVar[ST]  # value = <ST.sdf: 13>
    sphere: typing.ClassVar[ST]  # value = <ST.sphere: 1>
    ssBox: typing.ClassVar[ST]  # value = <ST.ssBox: 8>
    ssBoxElip: typing.ClassVar[ST]  # value = <ST.ssBoxElip: 10>
    ssCvx: typing.ClassVar[ST]  # value = <ST.ssCvx: 7>
    ssCylinder: typing.ClassVar[ST]  # value = <ST.ssCylinder: 9>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SY:
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
    __members__: typing.ClassVar[dict[str, SY]]  # value = {'touch': <SY.touch: 0>, 'above': <SY.above: 1>, 'inside': <SY.inside: 2>, 'oppose': <SY.oppose: 3>, 'restingOn': <SY.restingOn: 4>, 'poseEq': <SY.poseEq: 5>, 'positionEq': <SY.positionEq: 6>, 'stableRelPose': <SY.stableRelPose: 7>, 'stablePose': <SY.stablePose: 8>, 'stable': <SY.stable: 9>, 'stableOn': <SY.stableOn: 10>, 'dynamic': <SY.dynamic: 11>, 'dynamicOn': <SY.dynamicOn: 12>, 'dynamicTrans': <SY.dynamicTrans: 13>, 'quasiStatic': <SY.quasiStatic: 14>, 'quasiStaticOn': <SY.quasiStaticOn: 15>, 'downUp': <SY.downUp: 16>, 'stableZero': <SY.stableZero: 18>, 'contact': <SY.contact: 19>, 'contactStick': <SY.contactStick: 20>, 'contactComplementary': <SY.contactComplementary: 21>, 'bounce': <SY.bounce: 22>, 'push': <SY.push: 23>, 'magic': <SY.magic: 24>, 'magicTrans': <SY.magicTrans: 25>, 'pushAndPlace': <SY.pushAndPlace: 26>, 'topBoxGrasp': <SY.topBoxGrasp: 27>, 'topBoxPlace': <SY.topBoxPlace: 28>, 'dampMotion': <SY.dampMotion: 29>, 'identical': <SY.identical: 30>, 'alignByInt': <SY.alignByInt: 31>, 'makeFree': <SY.makeFree: 32>, 'forceBalance': <SY.forceBalance: 33>, 'relPosY': <SY.relPosY: 34>, 'touchBoxNormalX': <SY.touchBoxNormalX: 35>, 'touchBoxNormalY': <SY.touchBoxNormalY: 36>, 'touchBoxNormalZ': <SY.touchBoxNormalZ: 37>, 'boxGraspX': <SY.boxGraspX: 38>, 'boxGraspY': <SY.boxGraspY: 39>, 'boxGraspZ': <SY.boxGraspZ: 40>, 'lift': <SY.lift: 41>, 'stableYPhi': <SY.stableYPhi: 42>, 'stableOnX': <SY.stableOnX: 43>, 'stableOnY': <SY.stableOnY: 44>, 'end': <SY.end: 46>}
    above: typing.ClassVar[SY]  # value = <SY.above: 1>
    alignByInt: typing.ClassVar[SY]  # value = <SY.alignByInt: 31>
    bounce: typing.ClassVar[SY]  # value = <SY.bounce: 22>
    boxGraspX: typing.ClassVar[SY]  # value = <SY.boxGraspX: 38>
    boxGraspY: typing.ClassVar[SY]  # value = <SY.boxGraspY: 39>
    boxGraspZ: typing.ClassVar[SY]  # value = <SY.boxGraspZ: 40>
    contact: typing.ClassVar[SY]  # value = <SY.contact: 19>
    contactComplementary: typing.ClassVar[SY]  # value = <SY.contactComplementary: 21>
    contactStick: typing.ClassVar[SY]  # value = <SY.contactStick: 20>
    dampMotion: typing.ClassVar[SY]  # value = <SY.dampMotion: 29>
    downUp: typing.ClassVar[SY]  # value = <SY.downUp: 16>
    dynamic: typing.ClassVar[SY]  # value = <SY.dynamic: 11>
    dynamicOn: typing.ClassVar[SY]  # value = <SY.dynamicOn: 12>
    dynamicTrans: typing.ClassVar[SY]  # value = <SY.dynamicTrans: 13>
    end: typing.ClassVar[SY]  # value = <SY.end: 46>
    forceBalance: typing.ClassVar[SY]  # value = <SY.forceBalance: 33>
    identical: typing.ClassVar[SY]  # value = <SY.identical: 30>
    inside: typing.ClassVar[SY]  # value = <SY.inside: 2>
    lift: typing.ClassVar[SY]  # value = <SY.lift: 41>
    magic: typing.ClassVar[SY]  # value = <SY.magic: 24>
    magicTrans: typing.ClassVar[SY]  # value = <SY.magicTrans: 25>
    makeFree: typing.ClassVar[SY]  # value = <SY.makeFree: 32>
    oppose: typing.ClassVar[SY]  # value = <SY.oppose: 3>
    poseEq: typing.ClassVar[SY]  # value = <SY.poseEq: 5>
    positionEq: typing.ClassVar[SY]  # value = <SY.positionEq: 6>
    push: typing.ClassVar[SY]  # value = <SY.push: 23>
    pushAndPlace: typing.ClassVar[SY]  # value = <SY.pushAndPlace: 26>
    quasiStatic: typing.ClassVar[SY]  # value = <SY.quasiStatic: 14>
    quasiStaticOn: typing.ClassVar[SY]  # value = <SY.quasiStaticOn: 15>
    relPosY: typing.ClassVar[SY]  # value = <SY.relPosY: 34>
    restingOn: typing.ClassVar[SY]  # value = <SY.restingOn: 4>
    stable: typing.ClassVar[SY]  # value = <SY.stable: 9>
    stableOn: typing.ClassVar[SY]  # value = <SY.stableOn: 10>
    stableOnX: typing.ClassVar[SY]  # value = <SY.stableOnX: 43>
    stableOnY: typing.ClassVar[SY]  # value = <SY.stableOnY: 44>
    stablePose: typing.ClassVar[SY]  # value = <SY.stablePose: 8>
    stableRelPose: typing.ClassVar[SY]  # value = <SY.stableRelPose: 7>
    stableYPhi: typing.ClassVar[SY]  # value = <SY.stableYPhi: 42>
    stableZero: typing.ClassVar[SY]  # value = <SY.stableZero: 18>
    topBoxGrasp: typing.ClassVar[SY]  # value = <SY.topBoxGrasp: 27>
    topBoxPlace: typing.ClassVar[SY]  # value = <SY.topBoxPlace: 28>
    touch: typing.ClassVar[SY]  # value = <SY.touch: 0>
    touchBoxNormalX: typing.ClassVar[SY]  # value = <SY.touchBoxNormalX: 35>
    touchBoxNormalY: typing.ClassVar[SY]  # value = <SY.touchBoxNormalY: 36>
    touchBoxNormalZ: typing.ClassVar[SY]  # value = <SY.touchBoxNormalZ: 37>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Simulation:
    """
    A direct simulation interface to physics engines (Nvidia PhysX, Bullet) -- see https://marctoussaint.github.io/robotics-course/tutorials/simulation.html
    """
    def __init__(self, C: Config, engine: SimulationEngine, verbose: int = 2) -> None:
        """
        create a Simulation that is associated/attached to the given configuration
        """
    def addSensor(self, sensorName: str, width: int = 640, height: int = 360, focalLength: float = -1.0, orthoAbsHeight: float = -1.0, zRange: arr = []) -> ...:
        ...
    def attach(self, gripper: Frame, obj: Frame) -> None:
        ...
    def closeGripper(self, gripperFrameName: str, width: float = 0.05, speed: float = 0.3, force: float = 20.0) -> None:
        ...
    def depthData2pointCloud(self, arg0: numpy.ndarray[numpy.float32], arg1: list[float]) -> numpy.ndarray[numpy.float64]:
        ...
    def detach(self, obj: Frame) -> None:
        ...
    def getGripperIsGrasping(self, gripperFrameName: str) -> bool:
        ...
    def getGripperWidth(self, gripperFrameName: str) -> float:
        ...
    def getImageAndDepth(self) -> tuple:
        ...
    def getScreenshot(self) -> ...:
        ...
    def getState(self) -> tuple:
        """
        returns a 4-tuple or frame state, joint state, frame velocities (linear & angular), joint velocities
        """
    def getTimeToSplineEnd(self) -> float:
        ...
    def get_q(self) -> arr:
        ...
    def get_qDot(self) -> arr:
        ...
    def openGripper(self, gripperFrameName: str, width: float = 0.075, speed: float = 0.3) -> None:
        ...
    def pushConfigurationToSimulator(self, frameVelocities: arr = ..., jointVelocities: arr = ...) -> None:
        """
        set the simulator to the full (frame) state of the configuration
        """
    def resetSplineRef(self) -> None:
        """
        reset the spline reference, i.e., clear the current spline buffer and initialize it to constant spline at current position (to which setSplineRef can append)
        """
    def selectSensor(self, sensorName: str) -> ...:
        ...
    def setSplineRef(self, path: arr, times: arr, append: bool = True) -> None:
        """
        set the spline reference to generate motion
        * path: single configuration, or sequence of spline control points
        * times: array with single total duration, or time for each control point (times.N==path.d0)
        * append: append (with zero-velocity at append), or smoothly overwrite
        """
    def setState(self, frameState: arr, jointState: arr = ..., frameVelocities: arr = ..., jointVelocities: arr = ...) -> None:
        ...
    def step(self, u_control: arr, tau: float = 0.01, u_mode: ControlMode = ...) -> None:
        ...
class SimulationEngine:
    """
    Members:
    
      physx
    
      bullet
    
      kinematic
    """
    __members__: typing.ClassVar[dict[str, SimulationEngine]]  # value = {'physx': <SimulationEngine.physx: 1>, 'bullet': <SimulationEngine.bullet: 2>, 'kinematic': <SimulationEngine.kinematic: 3>}
    bullet: typing.ClassVar[SimulationEngine]  # value = <SimulationEngine.bullet: 2>
    kinematic: typing.ClassVar[SimulationEngine]  # value = <SimulationEngine.kinematic: 3>
    physx: typing.ClassVar[SimulationEngine]  # value = <SimulationEngine.physx: 1>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Skeleton:
    def __init__(self) -> None:
        ...
    def add(self, arg0: list) -> None:
        ...
    def addEntry(self, timeInterval: arr, symbol: SY, frames: StringA) -> None:
        ...
    def addExplicitCollisions(self, collisions: StringA) -> None:
        ...
    def addLiftPriors(self, lift: StringA) -> None:
        ...
    def enableAccumulatedCollisions(self, enable: bool = True) -> None:
        ...
    def getKomo_finalSlice(self, Configuration: Config, lenScale: float, homingScale: float, collScale: float) -> KOMO:
        ...
    def getKomo_path(self, Configuration: Config, stepsPerPhase: int, accScale: float, lenScale: float, homingScale: float, collScale: float) -> KOMO:
        ...
    def getKomo_waypoints(self, Configuration: Config, lenScale: float, homingScale: float, collScale: float) -> KOMO:
        ...
    def getMaxPhase(self) -> float:
        ...
    def getTwoWaypointProblem(self, t2: int, komoWays: KOMO) -> tuple:
        ...
class SolverReturn:
    """
    return of nlp solve call
    """
    done: bool
    eq: float
    evals: int
    f: float
    feasible: bool
    ineq: float
    sos: float
    time: float
    x: arr
    def __init__(self) -> None:
        ...
    def __str__(self) -> str:
        ...
    def dict(self) -> dict:
        ...
class TAMP_Provider:
    """
    TAMP_Provider
    """
def compiled() -> str:
    """
    return a compile date+time version string
    """
def default_Logic2KOMO_Translator() -> Logic2KOMO_Translator:
    ...
def default_TAMP_Provider(C: Config, lgp_config_file: str) -> TAMP_Provider:
    ...
def depthImage2PointCloud(depth: numpy.ndarray[numpy.float32], fxycxy: arr) -> arr:
    """
    return the point cloud from the depth image
    """
def params_add(*args, **kwargs) -> None:
    """
    add/set parameters
    """
def params_clear() -> None:
    """
    clear all parameters
    """
def params_file(filename: str) -> None:
    """
    add parameters from a file
    """
def params_print() -> None:
    """
    print the parameters
    """
def raiPath(arg0: str) -> ...:
    """
    get a path relative to rai base path
    """
def setRaiPath(arg0: str) -> None:
    """
    redefine the rai (or rai-robotModels) path
    """
_left: ArgWord  # value = <ArgWord._left: 0>
_path: ArgWord  # value = <ArgWord._path: 3>
_right: ArgWord  # value = <ArgWord._right: 1>
_sequence: ArgWord  # value = <ArgWord._sequence: 2>
