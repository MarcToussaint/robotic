import robotic as ry
import numpy as np
import time

class ManipulationModelling():

    def __init__(self, info=str()):
        self.komo = None
        self.info = info

    def setup_inverse_kinematics(self, C, homing_scale=1e-1, accumulated_collisions=True, joint_limits=True, quaternion_norms=False):
        '''
        setup a 1 phase single step problem
        '''
        assert self.komo==None
        self.komo = ry.KOMO(C, 1., 1, 0, accumulated_collisions)
        self.komo.addControlObjective([], 0, homing_scale)
        if accumulated_collisions:
            self.komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e0])
        if joint_limits:
            self.komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1e0])
        if quaternion_norms:
            self.komo.addQuaternionNorms()

    def setup_sequence(self, C, K, homing_scale, velocity_scale, accumulated_collisions, joint_limits, quaternion_norms):
        assert self.komo==None
        self.komo = ry.KOMO(C, K, 1, 1, accumulated_collisions)
        self.komo.addControlObjective([], 0, homing_scale)
        self.komo.addControlObjective([], 1, velocity_scale)
        if accumulated_collisions:
            self.komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e0])
        if joint_limits:
            self.komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1e0])
        if quaternion_norms:
            self.komo.addQuaternionNorms()

    def setup_motion(self, C, K, steps_per_phase, homing_scale, acceleration_scale, accumulated_collisions, joint_limits, quaternion_norms):
        assert self.komo==None
        self.komo = ry.KOMO(C, K, steps_per_phase, 2, accumulated_collisions)
        if homing_scale>0.:
            self.komo.addControlObjective([], 0, homing_scale)
        self.komo.addControlObjective([], 2, acceleration_scale)
        if accumulated_collisions:
            self.komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e0])
        if joint_limits:
            self.komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq, [1e0])
        if quaternion_norms:
            self.komo.addQuaternionNorms()

        # zero vel at end
        self.komo.addObjective([float(K)], ry.FS.qItself, [], ry.OT.eq, [1e0], [], 1)

    def setup_pick_and_place_waypoints(self, C, gripper, obj, homing_scale=1e-2, velocity_scale=1e-1, accumulated_collisions=True, joint_limits=True, quaternion_norms=False):
        '''
        setup a 2 phase pick-and-place problem, with a pick switch at time 1, and a place switch at time 2
        the place mode switch at the final time two might seem obselete, but this switch also implies the geometric constraints of placeOn
        '''
        assert self.komo==None
        self.setup_sequence(C, 2, homing_scale, velocity_scale, accumulated_collisions, joint_limits, quaternion_norms)

        #-- option 1: old-style mode switches: //a temporary free stable joint gripper -> object
        #self.komo.addModeSwitch([1.,-1.], ry.SY.stable, [gripper, obj], True)
        #-- option 2: a permanent free stable gripper->grasp joint; and a snap grasp->object
        self.add_stable_frame(ry.JT.free, gripper, 'obj_grasp', initFrame=obj)
        self.snap_switch(1., 'obj_grasp', obj)
        #-- option 3: a permanent free stable object->grasp joint; and a snap gripper->grasp
        # self.add_stable_frame(ry.JT.free, obj, 'obj_grasp', initFrame=obj)
        # self.snap_switch(1., gripper, 'obj_grasp')

    def setup_point_to_point_motion(self, C, q1, homing_scale=1e-2, acceleration_scale=1e-1, accumulated_collisions=True, joint_limits=True, quaternion_norms=False):
        '''
        setup a 1 phase fine-grained motion problem with 2nd order (acceleration) control costs
        '''
        assert self.komo==None
        self.setup_motion(C, 1, 32, homing_scale, acceleration_scale, accumulated_collisions, joint_limits, quaternion_norms)

        self.komo.initWithWaypoints([q1], 1, interpolate=True, qHomeInterpolate=.5, verbose=0)
        self.komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, scale=[1e0], target=q1)

    def setup_point_to_point_rrt(self, C, q0, q1, explicitCollisionPairs):
        rrt = ry.PathFinder()
        rrt.setProblem(C, q0, q1)
        if len(explicitCollisionPairs):
            rrt.setExplicitCollisionPairs(explicitCollisionPairs)

    def add_stable_frame(self, jointType, parent, name, initFrame=None, markerSize=-1.):
        if isinstance(initFrame, str):
            initFrame = self.komo.getConfig().getFrame(initFrame)
        f = self.komo.addStableFrame(name, parent, jointType, True, initFrame)
        if markerSize>0.:
            f.setShape(ry.ST.marker, [.2])
            f.setColor([1., 0., 1.])
        #f.joint.sampleSdv=1.
        #f.joint.setRandom(self.komo.timeSlices.d1, 0)

    def grasp_top_box(self, time, gripper, obj, grasp_direction='xz'):
        '''
        grasp a box with a centered top grasp (axes fully aligned)
        '''
        if grasp_direction == 'xz':
            align = [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ]
        elif grasp_direction == 'yz':
            align = [ry.FS.scalarProductYY, ry.FS.scalarProductXZ, ry.FS.scalarProductYZ]
        elif grasp_direction == 'xy':
            align = [ry.FS.scalarProductXY, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ]
        elif grasp_direction == 'zy':
            align = [ry.FS.scalarProductXX, ry.FS.scalarProductXZ, ry.FS.scalarProductZZ]
        elif grasp_direction == 'yx':
            align = [ry.FS.scalarProductYY, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ]
        elif grasp_direction == 'zx':
            align = [ry.FS.scalarProductYX, ry.FS.scalarProductYZ, ry.FS.scalarProductZZ]
        else:
            raise Exception('pickDirection not defined:', grasp_direction)

        # position: centered
        self.komo.addObjective([time], ry.FS.positionDiff, [gripper, obj], ry.OT.eq, [1e1])

        # orientation: grasp axis orthoginal to target plane X-specific
        self.komo.addObjective([time-.2,time], align[0], [obj, gripper], ry.OT.eq, [1e0])
        self.komo.addObjective([time-.2,time], align[1], [obj, gripper], ry.OT.eq, [1e0])
        self.komo.addObjective([time-.2,time], align[2], [obj, gripper], ry.OT.eq, [1e0])


    def grasp_box(self, time, gripper, obj, palm, grasp_direction='x', margin=.02):
        '''
        general grasp of a box, squeezing along provided grasp_axis (-> 3
        possible grasps of a box), where and angle of grasp is decided by
        inequalities on grasp plan and no-collision of box and palm
        '''
        if grasp_direction == 'x':
            xLine = np.array([[1, 0, 0]])
            yzPlane = np.array([[0, 1, 0],[0, 0, 1]])
            align = [ry.FS.scalarProductXY, ry.FS.scalarProductXZ]
        elif grasp_direction == 'y':
            xLine = np.array([[0, 1, 0]])
            yzPlane = np.array([[1, 0, 0],[0, 0, 1]])
            align = [ry.FS.scalarProductXX, ry.FS.scalarProductXZ]
        elif grasp_direction == 'z':
            xLine = np.array([[0, 0, 1]])
            yzPlane = np.array([[1, 0, 0],[0, 1, 0]])
            align = [ry.FS.scalarProductXX, ry.FS.scalarProductXY]
        else:
            raise Exception('grasp_direction not defined:', grasp_direction)

        boxSize = self.komo.getConfig().getFrame(obj).getSize()[:3]

        # position: center in inner target plane X-specific
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.eq, xLine*1e1)
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.ineq, yzPlane*1e1, .5*boxSize-margin)
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.ineq, yzPlane*(-1e1), -.5*boxSize+margin)

        # orientation: grasp axis orthoginal to target plane X-specific
        self.komo.addObjective([time-.2,time], align[0], [gripper, obj], ry.OT.eq, [1e0])
        self.komo.addObjective([time-.2,time], align[1], [gripper, obj], ry.OT.eq, [1e0])

        # no collision with palm
        self.komo.addObjective([time-.3,time], ry.FS.distance, [palm, obj], ry.OT.ineq, [1e1], [-.001])

    def grasp_cylinder(self, time, gripper, obj, palm, margin=.02):
        '''
        general grasp of a cylinder, with squeezing the axis normally,
        inequality along z-axis for positioning, and no-collision with palm
        '''
        size = self.komo.getConfig().getFrame(obj).getSize()[:2]

        # position: center along axis, stay within z-range
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.eq, np.array([[1, 0, 0],[0, 1, 0]])*1e1)
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.ineq, np.array([[0, 0, 1]])*1e1, np.array([0.,0.,.5*size[0]-margin]))
        self.komo.addObjective([time], ry.FS.positionRel, [gripper, obj], ry.OT.ineq, np.array([[0, 0, 1]])*(-1e1), np.array([0.,0.,-.5*size[0]+margin]))

        # orientation: grasp axis orthoginal to target plane X-specific
        self.komo.addObjective([time-.2,time], ry.FS.scalarProductXZ, [gripper, obj], ry.OT.eq, [1e0])

        # no collision with palm
        self.komo.addObjective([time-.3,time], ry.FS.distance, [palm, obj], ry.OT.ineq, [1e1], [-.001])

    def place_box(self, time, obj, table, palm, place_direction='z', margin=.02):
        '''
        placement of one box on another
        '''
        zVectorTarget = np.array([0.,0.,1.])
        obj_frame = self.komo.getConfig().getFrame(obj)
        boxSize = obj_frame.getSize()
        if obj_frame.getShapeType()==ry.ST.ssBox:
            boxSize = boxSize[:3]
        elif obj_frame.getType()==ry.ST.ssCylinder:
            boxSize = [boxSize[1], boxSize[1], boxSize[0]] 
        else:
            raise Exception('NIY')

        tableSize = self.komo.getConfig().getFrame(table).getSize()[:3]
        if place_direction == 'x':
            relPos = .5*(boxSize[0]+tableSize[2])
            zVector = ry.FS.vectorX
            align = [ry.FS.scalarProductXX, ry.FS.scalarProductYX]
        elif place_direction == 'y':
            relPos = .5*(boxSize[1]+tableSize[2])
            zVector = ry.FS.vectorY
            align = [ry.FS.scalarProductXY, ry.FS.scalarProductYY]
        elif place_direction == 'z':
            relPos = .5*(boxSize[2]+tableSize[2])
            zVector = ry.FS.vectorZ
            align = [ry.FS.scalarProductXZ, ry.FS.scalarProductYZ]
        elif place_direction == 'xNeg':
            relPos = .5*(boxSize[0]+tableSize[2])
            zVector = ry.FS.vectorX
            zVectorTarget *= -1.
            align = [ry.FS.scalarProductXX, ry.FS.scalarProductYX]
        elif place_direction == 'yNeg':
            relPos = .5*(boxSize[1]+tableSize[2])
            zVector = ry.FS.vectorY
            zVectorTarget *= -1.
            align = [ry.FS.scalarProductXY, ry.FS.scalarProductYY]
        elif place_direction == 'zNeg':
            relPos = .5*(boxSize[2]+tableSize[2])
            zVector = ry.FS.vectorZ
            zVectorTarget *= -1.
            align = [ry.FS.scalarProductXZ, ry.FS.scalarProductYZ]
        else:
            raise Exception('place_direction not defined:', place_direction)

        # position: above table, inside table
        self.komo.addObjective([time], ry.FS.positionDiff, [obj, table], ry.OT.eq, 1e1*np.array([[0, 0, 1]]), np.array([.0, .0, relPos]))
        self.komo.addObjective([time], ry.FS.positionRel, [obj, table], ry.OT.ineq, 1e1*np.array([[1, 0, 0],[0, 1, 0]]), .5*tableSize-margin)
        self.komo.addObjective([time], ry.FS.positionRel, [obj, table], ry.OT.ineq, -1e1*np.array([[1, 0, 0],[0, 1, 0]]), -.5*tableSize+margin)

        # orientation: Z-up
        self.komo.addObjective([time-.2, time], zVector, [obj], ry.OT.eq, [0.5], zVectorTarget)
        self.komo.addObjective([time-.2,time], align[0], [table, obj], ry.OT.eq, [1e0])
        self.komo.addObjective([time-.2,time], align[1], [table, obj], ry.OT.eq, [1e0])

        # no collision with palm
        if palm != None:
           self.komo.addObjective([time-.3, time], ry.FS.distance, [palm, table], ry.OT.ineq, [1e1], [-.001])

    def straight_push(self, time_interval, obj, gripper, table):
        #start & end helper frames
        helperStart = f'_straight_pushStart_{gripper}_{obj}_{time_interval[0]}'
        helperEnd = f'_straight_pushEnd_{gripper}_{obj}_{time_interval[1]}'
        if not self.komo.getConfig().getFrame(helperStart, False):
            self.add_stable_frame(ry.JT.hingeZ, table, helperStart, obj, .3)
        if not self.komo.getConfig().getFrame(helperEnd, False):
            self.add_stable_frame(ry.JT.transXYPhi, table, helperEnd, obj, .3)

        #-- couple both frames symmetricaly
        #aligned orientation
        self.komo.addObjective([time_interval[0]], ry.FS.vectorYDiff, [helperStart, helperEnd], ry.OT.eq, [1e1])
        #aligned position
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [helperEnd, helperStart], ry.OT.eq, 1e1*np.array([[1., 0., 0.], [0., 0., 1.]]))
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [helperStart, helperEnd], ry.OT.eq, 1e1*np.array([[1., 0., 0.], [0., 0., 1.]]))
        #at least 2cm appart, positivenot !not  direction
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [helperEnd, helperStart], ry.OT.ineq, -1e2*np.array([[0., 1., 0.]]), [.0, .02, .0])
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [helperStart, helperEnd], ry.OT.ineq, 1e2*np.array([[0., 1., 0.]]), [.0, -.02, .0])

        #gripper touch
        self.komo.addObjective([time_interval[0]], ry.FS.negDistance, [gripper, obj], ry.OT.eq, [1e1], [-.02])
        #gripper start position
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [gripper, helperStart], ry.OT.eq, 1e1*np.array([[1., 0., 0.], [0., 0., 1.]]))
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [gripper, helperStart], ry.OT.ineq, 1e1*np.array([[0., 1., 0.]]), [.0, -.02, .0])
        #gripper start orientation
        self.komo.addObjective([time_interval[0]], ry.FS.scalarProductYY, [gripper, helperStart], ry.OT.ineq, [-1e1], [.2])
        self.komo.addObjective([time_interval[0]], ry.FS.scalarProductYZ, [gripper, helperStart], ry.OT.ineq, [-1e1], [.2])
        self.komo.addObjective([time_interval[0]], ry.FS.vectorXDiff, [gripper, helperStart], ry.OT.eq, [1e1])

        #obj end position
        self.komo.addObjective([time_interval[1]], ry.FS.positionDiff, [obj, helperEnd], ry.OT.eq, [1e1])
        #obj end orientation: unchanged
        self.komo.addObjective([time_interval[1]], ry.FS.quaternion, [obj], ry.OT.eq, [1e1], [], 1); #qobjPose.rot.getArr4d())
    
        return helperStart

    def pull(self, times, obj, gripper, table):
        self.add_stable_frame(ry.JT.transXYPhi, table, '_pull_end', obj)
        self.komo.addObjective([times[0]], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e1], np.array([0,0,1]))
        self.komo.addObjective([times[1]], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e1], np.array([0,0,1]))
        self.komo.addObjective([times[0]], ry.FS.vectorZ, [obj], ry.OT.eq, [1e1], np.array([0,0,1]))
        self.komo.addObjective([times[1]], ry.FS.vectorZ, [obj], ry.OT.eq, [1e1], np.array([0,0,1]))
        self.komo.addObjective([times[1]], ry.FS.positionDiff, [obj, '_pull_end'], ry.OT.eq, [1e1])
        self.komo.addObjective([times[0]], ry.FS.positionRel, [gripper, obj], ry.OT.eq, 1e1*np.array([[1., 0., 0.], [0., 1., 0.]]), np.array([0, 0, 0]))
        self.komo.addObjective([times[0]], ry.FS.negDistance, [gripper, obj], ry.OT.eq, [1e1], [-.005])

    def no_collisions(self, time_interval, objs, margin=.001):
        '''
        inequality on distance between multiple objects
        '''
        while len(objs) > 1:
            comp = objs[0]
            del objs[0]
            for obj in objs:
                self.komo.addObjective(time_interval, ry.FS.negDistance, [comp, obj], ry.OT.ineq, [1e1], [-margin])

    def snap_switch(self, time, parent, obj):
        '''
        a kinematic mode switch, where at given time the obj becomes attached to parent with zero relative transform
        the parent is typically a stable_frame (i.e. a frame that has parameterized but stable (i.e. constant) relative transform)
        '''
        self.komo.addRigidSwitch(time, [parent, obj])

    def switch_place():
        '''
        a kinematic mode switch, where obj becomes attached to table, with a 3D parameterized (XYPhi) stable relative pose
        this requires obj and table to be boxes and assumes default placement alone z-axis
        more general placements have to be modelled with switch_pick (table picking the object) and additinal user-defined geometric constraints
        '''

    def target_position():
        '''
        impose a specific 3D target position on some object
        '''

    def target_relative_xy_position(self, time, obj, relativeTo, pos):
        '''
        impose a specific 3D target position on some object
        '''
        if len(pos)==2:
            pos.append(0.)
        self.komo.addObjective([time], ry.FS.positionRel, [obj, relativeTo], ry.OT.eq, scale=1e1*np.array([[1,0,0],[0,1,0]]), target=pos)

    def target_x_orientation(self, time, obj, x_vector):
        '''
        '''
        self.komo.addObjective([time], ry.FS.vectorX, [obj], ry.OT.eq, scale=[1e1], target=x_vector)

    def bias(self, time, qBias, scale=1e0):
        '''
        impose a square potential bias directly in joint space
        '''
        self.komo.addObjective([time], ry.FS.qItself, [], ry.OT.sos, scale=scale, target=qBias)

    def retract(self, time_interval, gripper, dist=.03):
        helper = f'_{gripper}_retract_{time_interval[0]}'
        f = self.komo.getFrame(gripper, time_interval[0])
        self.add_stable_frame(ry.JT.none, '', helper, f)
    #  self.komo.view(True, helper)

        self.komo.addObjective(time_interval, ry.FS.positionRel, [gripper, helper], ry.OT.eq, 1e2 * np.array([[1, 0, 0]]))
        self.komo.addObjective(time_interval, ry.FS.quaternionDiff, [gripper, helper], ry.OT.eq, [1e2])
        self.komo.addObjective([time_interval[1]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, -1e2 * np.array([[0, 0, 1]]), target = [0., 0., dist])

    def approach(self, time_interval, gripper, dist=.03):
        helper = f'_{gripper}_approach_{time_interval[1]}'
        f = self.komo.getFrame(gripper, time_interval[1])
        self.add_stable_frame(ry.JT.none, '', helper, f)
    #  self.komo.view(True, helper)

        self.komo.addObjective(time_interval, ry.FS.positionRel, [gripper, helper], ry.OT.eq, 1e2 * np.array([[1, 0, 0]]))
        self.komo.addObjective(time_interval, ry.FS.quaternionDiff, [gripper, helper], ry.OT.eq, [1e2])
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, -1e2 * np.array([[0, 0, 1]]), target = [0., 0., dist])

    def retractPush(self, time_interval, gripper, dist):
        helper = f'_{gripper}_retractPush_{time_interval[0]}'
        f = self.komo.getFrame(gripper, time_interval[0])
        self.add_stable_frame(ry.JT.none, '', helper, f)
    #  self.komo.addObjective(time_interval, ry.FS.positionRel, [gripper, helper], ry.OT.eq, * np.array([[1,3},{1,0,0]]))
        #  self.komo.addObjective(time_interval, ry.FS.quaternionDiff, [gripper, helper], ry.OT.eq, [1e2])
        self.komo.addObjective(time_interval, ry.FS.positionRel, [gripper, helper], ry.OT.eq, * np.array([[1, 0, 0]]))
        self.komo.addObjective([time_interval[1]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, * np.array([[0, 1, 0]]), [0., -dist, 0.])
        self.komo.addObjective([time_interval[1]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, -1e2 * np.array([[0, 0, 1]]), [0., 0., dist])

    def approachPush(self, time_interval, gripper, dist):
        helper = f'_{gripper}_approachPush_{time_interval[1]}'
        f = self.komo.getFrame(gripper, time_interval[1])
        self.add_stable_frame(ry.JT.none, '', helper, f)
        self.komo.addObjective(time_interval, ry.FS.positionRel, [gripper, helper], ry.OT.eq, * np.array([[1, 0, 0]]))
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, * np.array([[0, 1, 0]]), [0., -dist, 0.])
        self.komo.addObjective([time_interval[0]], ry.FS.positionRel, [gripper, helper], ry.OT.ineq, -1e2 * np.array([[0, 0, 1]]), [0., 0., dist])
        
    def solve(self, verbose=1):
        if self.komo:
            sol = ry.NLP_Solver()
            sol.setProblem(self.komo.nlp())
            sol.setOptions(damping=1e-1, verbose=verbose-1, stopTolerance=1e-3, maxLambda=100., stopInners=20, stopEvals=200)
            self.ret = sol.solve()
            if self.ret.feasible:
                self.path = self.komo.getPath()
            else:
                self.path = None
            if verbose>0:
                if not self.ret.feasible:
                    print(f'  -- infeasible:{self.info}\n     {self.ret}')
                    if verbose>1:
                        print(self.komo.report(False, True))
                        self.komo.view(True, f'failed: {self.info}\n{self.ret}')
                    if verbose>2:
                        while(self.komo.view_play(True, 1.)):
                            pass
                else:
                    print(f'  -- feasible:{self.info}\n     {self.ret}')
                    if verbose>2:
                        self.komo.view(True, f'success: {self.info}\n{self.ret}')
                    if verbose>3:
                        while(self.komo.view_play(True, 1.)):
                            pass

        elif self.rrt:
            ret = self.rrt.solve()
            if ret.feasible:
                self.path = ret.x
            else:
                self.path = None

        else:
            raise Exception('no problem defined')
            
        return self.path

    def debug(self, listObjectives, plotOverTime):
    #     cout <<'  -- DEBUG: ' <<info <<endl
    #     cout <<'  == solver return: ' <<*ret <<endl
    #     cout <<'  == all KOMO objectives with increasing errors:\n' <<self.komo.report(False, listObjectives, plotOverTime) <<endl
    # #  cout <<'  == objectives sorted by error and Lagrange gradient:\n' <<sol.reportLagrangeGradients(self.komo.featureNames) <<endl
    #     cout <<'  == view objective errors over slices in gnuplot' <<endl
    #     cout <<'  == scroll through solution in display window using SHIFT-scroll' <<endl
        self.komo.view(True, f'debug: {info}\n{self.ret}')

    def play(self, C, duration=1.):
        for t in range(self.path.shape[0]):
            C.setJointState(self.path[t])
            C.view(False, f'step {t}\n{self.info}')
            time.sleep(duration/self.path.shape[0])

    def sub_motion(self, phase, fixEnd=True, homing_scale=1e-2, acceleration_scale=1e-1, accumulated_collisions=True, quaternion_norms=False):
        (C, q0, q1) = self.komo.getSubProblem(phase)
        manip = ManipulationModelling(f'sub_motion_{phase}--{self.info}')
        manip.setup_point_to_point_motion(C, q1, homing_scale, acceleration_scale, accumulated_collisions, quaternion_norms)
        return manip

    def sub_rrt(self, phase, explicitCollisionPairs=[]):
        (C, q0, q1) = self.komo.getSubProblem(phase)
        manip = ManipulationModelling(f'sub_rrt_{phase}--{self.info}')
        manip.setup_point_to_point_rrt(C, q0, q1, explicitCollisionPairs)
        return manip
    
    @property
    def feasible(self):
        return self.ret.feasible
    
