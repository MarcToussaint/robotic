import numpy as np
import os

from collections import namedtuple
from KOMOcsail.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion, quaternion_from_euler

Objective = namedtuple('Objective', ['steps', 'type', 'symbols', 'parameters'])

BASE = 'base'
RIGHT_ARM = 'armR'
LEFT_ARM = 'armL'

JOINT_GROUPS = {
    BASE: ['worldTranslationRotation:{}'.format(i) for i in range(0, 3)],
    RIGHT_ARM: ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 
                 'r_elbow_flex_joint',  'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
    LEFT_ARM: ['l_shoulder_pan_joint',  'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 
                'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
}

GRASP_FRAMES = {
    RIGHT_ARM: 'pr2R',
    LEFT_ARM: 'pr2L',
}

FRAME_PREFIX_FROM_ARM = {
    RIGHT_ARM: 'r_',
    LEFT_ARM: 'l_',    
}


#######################################################

def running_remotely():
    return 'SSH_CONNECTION' in os.environ

#######################################################

def get_position_joints(body):
    return ['{}:{}'.format(body, i) for i in range(0, 3)]

def get_orientation_joints(body):
    return ['{}:{}'.format(body, i) for i in range(3, 7)]

def get_pose_joints(body):
    return get_position_joints(body) + get_orientation_joints(body)

def get_arm_frames(komo, arm):
    # TODO: pr2Frames
    return [f for f in komo.getFrameNames() if f.startswith(FRAME_PREFIX_FROM_ARM[f])]

#######################################################

def from_rai_conf(conf):
    return conf[1]

def from_rai_traj(traj):
    shape, values = traj
    length, dim = shape
    path = []
    for i in range(0, length*dim, dim):
        path.append(values[i:i+dim])
    return path

def to_rai_conf(conf):
    shape = [len(conf)]
    return (shape, conf)

def to_rai_traj(traj):
    length = len(traj)
    assert(2 <= length)
    dim = len(traj[0])
    data = []
    for conf in traj:
        assert(len(conf) == dim)
        data.extend(conf)
    shape = [length, dim]
    return (shape, data)

#######################################################

# setAbstractTask, setSkeleton

def get_conf(komo, joints=None):
    if joints is None:
        values = komo.getFullState()
    else:
        values = komo.getJointState(joints)
    return np.array(from_rai_conf(values))

def set_conf(komo, values, joints=None):
    rai_values = to_rai_conf(values)
    if joints is None:
        komo.setFullState(rai_values)
    else:
        #komo.setJointState(joints, rai_values)
        # TODO: bug in how joints are named
        current_state = get_conf(komo)
        joint_order = komo.getJointNames()
        index_from_joint = dict(zip(joint_order, range(len(joint_order))))
        for joint, value in zip(joints, values):
            current_state[index_from_joint[joint]] = value
        set_conf(komo, current_state)

def get_state(komo):
    return dict(zip(komo.getJointNames(), from_rai_conf(komo.getFullState())))

def get_world_pose(komo, frame):
    return from_rai_conf(komo.getWorldPose(frame))

#######################################################

def invert_tform(tform):
    return np.linalg.inv(tform)

def multiply_tforms(*tforms):
    result = np.eye(4)
    for tform in tforms:
        result = result.dot(tform)
    return result

def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return np.concatenate([point, quat_from_euler(euler)])

#######################################################

def quat_from_euler(euler):
    return quaternion_from_euler(*euler)

euler_from_quat = euler_from_quaternion

def point_from_pose(pose):
    return pose[:3]

def quat_from_pose(pose):
    return pose[3:]

def tform_from_pose(pose):
    tform = quaternion_matrix(pose[3:])
    tform[:3,3] = pose[:3]
    return tform

def pose_from_tform(tform):
    point = tform[:3,3]
    quat = quaternion_from_matrix(tform)
    return np.concatenate([point, quat])
