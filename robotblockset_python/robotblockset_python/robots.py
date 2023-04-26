import sys, os
sys.path.append(os.path.dirname(__file__))

from abc import ABC, abstractmethod, ABCMeta
import numpy as np
from time import perf_counter, sleep
from time import time as tm
import copy

from validateargs import *
from transformations import *
from tools import grad
from grippers import isgripper

from robotblockset_python.paths.min_jerk_trajectory import generate_min_jerk
from robotblockset_python.paths.trajectories import *

from scipy.interpolate import splrep, splev, make_interp_spline, CubicSpline, UnivariateSpline

import rospy

class _actual():
    def __init__(self):
        self.q = None
        self.qdot = None
        self.trq = None
        
        self.p = None
        self.R = None
        self.v = None
        
        self.FT = None
        self.trqExt = None

        self.x = None
        
class _command():
    def __init__(self):
        self.q = None
        self.qdot = None
        self.trq = None
        
        self.p = None
        self.R = None
        self.v = None
        
        self.FT = None
        self.mode = None
        self.data = None

        self.x = None
        
class robot():
    def __init__(self, **kwargs):
    
        __metaclass__ = ABCMeta #
        
        self.tsamp = 0.01        # sampling rate
        self.TCP = np.eye(4)     # robot TCP transformation matrix
        self.TBase = np.eye(4)   # robot base transformation matrix
        self.TObject = np.eye(4) # object transformation matrix
        self.Load = {'mass': 0,'COM': np.zeros(3),'inertia': np.zeros((3,3))} # Load
        self.gripper = None      # gripper object attached to robot
        self.FTsensor = None     # F/T sensor attached to robot
        self.user = None         # user data or object
        self.Tag = None

        self._t0 = 0             # initial time
        self._tt = 0             # actual time
        self._last_update = -100
        self._last_control_time = -100
        self._command_int = _command()  # Commanded values
        self._actual_int = _actual()            # Measured values
        self._do_update = True              # Enables state update and optional callback
        self._do_capture = False            # Enables calling calback function
        self._capture_callback = None       # Callback function in Update
        self._do_motion_check = False       # Enables checks during motion
        self._motion_check_callback = None  # Callback executed during motion
        self._control_strategy = 'JointPosition'  # Control strategy
        self._verbose = 2                   # verbose level
        
        self.nj = 7
        self.Init()
        
        #self.Default_TaskSpace = 'World'
        
        self.last_warning_print_time = tm() # When we reach joint limits, we don't want to get 100 warnings per second.
        
        # Performance stuff ( pre-creating arrays etc)
        
    def Init(self):
        
        self.InitObject()
        #self.ResetCurrentTarget()
        
    def InitObject(self):
        self._command_int.q = np.zeros((self.nj,1))
        self._command_int.q=np.zeros((self.nj,1))
        self._command_int.qdot=np.zeros((self.nj,1))
        self._command_int.trq=np.zeros((self.nj,1))
        self._command_int.p=np.zeros((3,1))
        self._command_int.R=np.eye(3)
        self._command_int.v=np.zeros((6,1))           
        self._command_int.FT=np.zeros((6,1))            
        #self._command_int.data=nan;            
        self._actual_int.q=np.zeros((self.nj,1))
        self._actual_int.qdot=np.zeros((self.nj,1))
        self._actual_int.trq=np.zeros((self.nj,1))
        self._actual_int.p=np.zeros((3,1))
        self._actual_int.R=np.eye(3)
        self._actual_int.v=np.zeros((6,1))
        self._actual_int.FT=np.zeros((6,1))
        self._actual_int.trqExt=np.zeros((self.nj,1))
        #self._Default_AddedTrq=zeros(self.nj,1);
        #self._Default_AddedFT=zeros(6,1);
        
        self.Default_State = 'Actual'
        self.Default_TaskSpace = 'World'
        self.Default_TaskPoseForm = 'TransformationMatrix'
        self.Default_TaskErrForm = 'Task'
        self.Default_TaskVelForm = 'Twist'
        self.Default_TaskFTForm = 'Wrench'
        self.Default_Kinematics = 'Robot'
        self.Default_TCPFrame = 'Gripper'
        self.Default_Source = 'Robot'
        self.Default_Strategy = 'JointPosition'
        self.Default_NullSpaceTask = 'JointLimits'
        self.Default_Direction = 'Forward'
        self.Default_RotDir = 'Short'
        self.Default_Traj = 'Poly'
        self.Default_TaskDOF = [1,1, 1, 1, 1, 1]
        self.Default_PosErr = 0.002
        self.Default_OriErr = 0.02
        self.Default_AddedTrq = 0            # Added joint torques
        self.Default_AddedFT = np.zeros((6,1))    # Added end-effector FT
        self.Default_Kp = 10   # Kinematic controller: position P gain
        self.Default_Kff = 1   # Kinematic controller: velocity FF gain
        self.Default_Kns = 10 # Kinematic controller: null-space gain
        self.Default_Kns0 = 0.1  #Kinematic controller: null-space gain for joint limits
        self.Default_Wait = 0.02
        self.Default_UpdateTime = 1.0
        
        self.Expected_State = ['Actual','Commanded']
        self.Expected_TaskSpace = ['World','Robot','Tool','Object']
        self.Expected_TaskPoseForm = ['TransformationMatrix','Pose','Quaternion','Euler','RotationMatrix','Position','RPY']
        self.Expected_TaskErrForm = ['Task','Position','Orientation']
        self.Expected_TaskVelForm = ['Twist','Linear','Angular']
        self.Expected_TaskFTForm = ['Wrench','Force','Torque']
        self.Expected_Kinematics = ['Calculated','Robot']
        self.Expected_TCPFrame = ['Gripper','Robot']
        self.Expected_Source = ['Robot','External']
        self.Expected_NullSpaceTask = ['None','Manipulability','JointLimits','ConfOptimization','TrackPath','PoseOptimization','TaskVelocity','JointVelocity']
        self.Expected_Direction = ['Forward','Backward']
        self.Expected_RotDir = ['Short','Long']
        self.Expected_Traj = ['Poly','Trap']
        
        return 0

    def jointvar(self, x):
        x = np.asarray(x)
        if x.shape[-1]==self.nj:
            return x
        else:
            TypeError('Parameter does not have proper shape')

    def spatial(self, x):
        x = np.asarray(x)
        if x.shape==(7,) or x.shape==(4, 4) or x.shape==(3,) or \
            x.shape==(4,) or x.shape==(3, 3) or x.shape==(6,):
            return x
        elif x.shape==(3, 4):
            x = np.vstack(x,np.array([0, 0, 0, 1]))
            return x
        else:
            TypeError('Parameter does not have proper shape')

    def simtime(self):
        return perf_counter()

    def sinhro_control(self, wait):
        dt = (self.simtime()-self._last_control_time)
        # print('Slow: ',dt, wait-dt)
        sleep(max(0,wait-dt))

    def ResetTime(self):
        self._tt = self.simtime()
        self._t0 = self._tt

    @abstractmethod
    def GetState(self):
        pass

    @abstractmethod
    def GoTo_q(self, q, qdot, trq, wait):
        pass

    @abstractmethod
    def kinmodel(self, *args, tcp=None, out='x'):
        pass

    @abstractmethod
    def jacobi(self, *args, tcp=None):
        pass

    @abstractmethod
    def GetJointStiffness(self):
        pass

    @abstractmethod
    def SetJointStiffness(self, stiffness, **kwargs):
        pass

    @abstractmethod
    def GetJointDamping(self):
        pass

    @abstractmethod
    def SetJointDamping(self, damping, **kwargs):
        pass

    @abstractmethod
    def SetJointSoft(self, softness, **kwargs):
        pass

    @abstractmethod
    def GetCartesianStiffness(self):
        pass

    @abstractmethod
    def SetCartesianStiffness(self, stiffness, **kwargs):
        pass

    @abstractmethod
    def GetCartesianDamping(self):
        pass

    @abstractmethod
    def SetCartesianDamping(self, damping, **kwargs):
        pass

    @abstractmethod
    def SetCartesianSoft(self, softness, **kwargs):
        pass

    @property
    def t(self):
        return self._tt-self._t0

    @property
    def Time(self):
        self.GetState()
        return self.t

    @property
    def command(self):
        return copy.deepcopy(self._command)

    @property
    def actual(self):
        return copy.deepcopy(self._actual)

    @property
    def q(self):
        return copy.deepcopy(self._actual_int.q)

    @property
    def qdot(self):
        return copy.deepcopy(self._actual_int.qdot)

    @property
    def x(self):
        return copy.deepcopy(self._actual_int.x)

    @property
    def p(self):
        return self._actual_int.x[:3]

    @property
    def Q(self):
        return self._actual_int.x[3:]

    @property
    def R(self):
        return q2r(self._actual_int.x[3:])

    @property
    def T(self):
        return x2t(self._actual_int.x)

    @property
    def v(self):
        return copy.deepcopy(self._actual_int.v)

    @property
    def pdot(self):
        return self._actual_int.v[:3]

    @property
    def w(self):
        return self._actual_int.v[3:]

    @property
    def trq(self):
        return copy.deepcopy(self._actual_int.trq)

    @property
    def trqExt(self):
        return copy.deepcopy(self._actual_int.trqExt)

    @property
    def FT(self):
        return copy.deepcopy(self._actual_int.FT)

    @property
    def F(self):
        return self._actual_int.FT[:3]
    @property
    def Trq(self):
        return self._actual_int.FT[3:]

    @property
    def q_ref(self):
        return copy.deepcopy(self._command_int.q)

    @property
    def qdot_ref(self):
        return copy.deepcopy(self._command_int.qdot)

    @property
    def x_ref(self):
        return copy.deepcopy(self._command_int.x)

    @property
    def p_ref(self):
        return self._command_int.x[:3]

    @property
    def Q_ref(self):
        return self._command_int.x[3:]

    @property
    def R_ref(self):
        return q2r(self._command_int.x[3:])

    @property
    def T_ref(self):
        return x2t(self._command_int.x)

    @property
    def v_ref(self):
        return copy.deepcopy(self._command_int.v)

    @property
    def pdot_ref(self):
        return self._command_int.v[:3]

    @property
    def w_ref(self):
        return self._command_int.v[3:]

    @property
    def FT_ref(self):
        return copy.deepcopy(self._command_int.FT)

    @property
    def F_ref(self):
        return self._command_int.FT[:3]
    @property
    def Trq_ref(self):
        return self._command_int.FT[3:]

    @property
    def q_err(self):
        return self.q_ref-self.q

    @property
    def qdot_err(self):
        return self.qdot_ref-self.qdot

    @property
    def x_err(self):
        return xerr(self.x_ref,self.x)

    @property
    def p_err(self):
        return self.p_ref-self.p

    @property
    def Q_err(self):
        return qerr(self.Q_ref,self.Q)

    @property
    def R_err(self):
        return self.R_ref@self.R.T

    @property
    def T_err(self):
        return terr(self.T_ref,self.T)

    @property
    def v_err(self):
        return self.v_ref-self.v

    @property
    def pdot_err(self):
        return self.pdot_ref-self.pdot

    @property
    def w_err(self):
        return self.w_ref-self.w

    def Update(self):
        if self._do_update:
            self.GetState()
            if self._do_capture and  self._capture_callback is not None:
                self._capture_callback(self)

    def EnableUpdate(self):
        self._do_update = True

    def DisableUpdate(self):
        self._do_update = False

    def GetUpdateStatus(self):
        return self._do_update

    def GetJointPos(self, state='Actual'):
        assert state in ['Actual', 'Commanded']
        if check_option(state, 'Actual'):
            return copy.deepcopy(self._actual_int.q)
        elif check_option(state, 'Commanded'):
            return copy.deepcopy(self._command_int.q)
        else:
            raise ValueError(f'State {state} not supported')

    def GetJointVel(self, state='Actual'):
        if check_option(state, 'Actual'):
            return copy.deepcopy(self._actual_int.qdot)
        elif check_option(state, 'Commanded'):
            return copy.deepcopy(self._command_int.qdot)
        else:
            raise ValueError(f'State {state} not supported')

    def GetPose(self, task_space='World', kinematics='Calculated', state='Actual', out='x', **kwargs):
        assert state in ['Actual', 'Commanded']
        assert kinematics in ['Calculated', 'Robot']
        
        #rospy.loginfo("GetPose state: {}, kinematics: {}".format(state,kinematics))
        
        if check_option(kinematics, 'Calculated'):
            x, _ = self.kinmodel(q = self.GetJointPos(state=state))
        elif check_option(kinematics, 'Robot'):
            if check_option(state, 'Actual'):
                x = copy.deepcopy(self._actual_int.x)
            elif check_option(state, 'Commanded'):
                x = copy.deepcopy(self._command_int.x)
            else:
                raise ValueError(f'State {state} not supported')
        else:
            raise ValueError(f'Kinematics calculation {kinematics} not supported')
        
        if check_option(task_space, 'World'):
            x = self.BaseToWorld(x)
        elif check_option(task_space, 'Object'):
            x = self.BaseToWorld(x)
            x = self.WorldToObject(x)
        elif check_option(task_space, 'Robot'):
            pass
        else:
            raise ValueError(f'Task space {task_space} not supported')
        return map_pose(x=x, out=out)

    def GetPos(self, task_space='World', kinematics='Calculated', state='Actual', **kwargs):
        return self.GetPose(task_space=task_space, kinematics=kinematics, state=state, out='p')

    def GetOri(self, task_space='World', kinematics='Calculated', state='Actual', out='Q', **kwargs):
        if out=='Q' or out=='R':
            return self.GetPose(task_space=task_space, kinematics=kinematics, state=state, out=out)
        else:
            raise ValueError(f'Output form {out} not supported')

    def GoToActual(self):
        self.Start()
        self.GoTo_q(self.GetJointPos(),np.zeros(self.nj),np.zeros(self.nj),1)
        self.Stop()

    #def ResetCurrentTarget(self):
    #    self.GetState()
    #
    #    self._command_int.q = copy.deepcopy(self._actual_int.q)
    #    self._command_int.qdot = copy.deepcopy(self._actual_int.qdot*0)
    #    self._command_int.trq = copy.deepcopy(self._actual_int.qdot*0)
    #    self._command_int.p = copy.deepcopy(self._actual_int.p)
    #    self._command_int.R = copy.deepcopy(self._actual_int.R)
    #    self._command_int.x = copy.deepcopy(self._actual_int.x)
    #    self._command_int.v = copy.deepcopy(self._actual_int.v*0)
    #    # self._command_int.q = self._actual_int.q
    #    # self._command_int.qdot = self._actual_int.qdot*0
    #    # self._command_int.trq = self._actual_int.qdot*0
    #    # self._command_int.p = self._actual_int.p
    #    # self._command_int.R = self._actual_int.R
    #    # self._command_int.v = np.zeros((6,1))
    #    self._command_int.FT = np.zeros((6,1))

    def ResetTaskTarget(self):
        xm = self.kinmodel(q = self._command_int.q)
        self._command_int.x = xm[0]
        self._command_int.v = xm[-1]@self._command_int.qdot
        
    def safety_check(self):
        """ Check
        - that the robot is not moving
        - that robot actual and commanded positions are close"""
        
        # Otherwise, for example move_until_contact wouldn't work
        if self._control_strategy in ['JointPositionTrajectory']:
            self.WaitUntilStopped(eps=2e-3)
        
        # Hack to prevent shaking. If some function forgets to set the _command_int.q, big problems occur here
        self.GetState()
        a = np.array(self._actual_int.q)
        b = np.array(self._command_int.q)
        q_error = np.abs(a-b)
        q_error_sum = np.sum(q_error)
        reset = 0
        if q_error_sum > 0.02:
            reset = 1
            self.ResetCurrentTarget()
            rospy.loginfo("Safety check resetting current target")

    def JMove(self, q, t, trq=None, traj='poly', wait=None, **kwargs):
        
        #Hack to use the action server implementation, however then the CMove wont work well with joint controller.
        #if self._control_strategy in ['JointImpedanceAction']:
            #self.SendJointTrapVelGoal(q = q, **kwargs)
            #return 0
        
        if self._control_strategy in ['CartesianImpedance', 'CartesianPose', 'CartesianVelocity']:
            x = self.kinmodel(q = q, out = 'x')[0]
            self.CMove(x = x, t=t)
            rospy.loginfo("Calling JMove but in CartesianImpedance strategy! Joint config will not be reached, only EE pos.")
            return 0
        
        kwargs.setdefault('qdot_max_factor', 1) # If not specified, maximal robot accelerations are allowed
        kwargs.setdefault('qddot_max_factor', 1)
        
        assert 0 < kwargs['qdot_max_factor'] <= 1
        assert 0 < kwargs['qddot_max_factor'] <= 1
        
        qdot_max = self.qdot_max * kwargs['qdot_max_factor']
        qddot_max = self.qddot_max * kwargs['qddot_max_factor']
        
        self.safety_check()
        
        q = self.jointvar(q)
        if self.CheckJointLimits(q):
            raise ValueError(f'Joint positions out of range')
        if not isscalar(t) or t<=0:
            raise ValueError(f'Time must be non-negative scalar')
        if wait is None:
            wait = self.tsamp
        if trq is None:
            trq = np.zeros(self.nj)
        else:
            trq = vector(trq, dim=self.nj)
            
        self.Message('JMove started', 2)
        self._command_int.mode = 1
        self.Start()
        tmperr = 0
        if t<=2*self.tsamp:
            self.GoTo_q(q, np.zeros(self.nj), trq, wait)
        else:
            time=np.arange(self.tsamp,t,self.tsamp)
            
            self.GetState()
            q0 = self.GetJointPos(state='Commanded')
            #rospy.loginfo("{} q_com: {}\nq_act: {}".format(self.Name, q0, self.GetJointPos(state='Actual')))
            #rospy.loginfo("\n{} dq: {}".format(self.Name, np.array(q0) - np.array(self.GetJointPos(state='Actual'))))

            qi, qdoti, qddoti = jtraj(q0, q, time, traj=traj)
            
            #Check for max velocities
            fac = np.max(np.max(np.abs(qdoti), axis=0)/qdot_max)
            if fac>1:
                rospy.loginfo("JMove extending time due to velocity qdot_max by factor {}".format(fac))
                t = time[-1] * fac
                time=np.arange(self.tsamp,t,self.tsamp)
                
                qi, qdoti, qddoti = jtraj(q0, q, time, traj=traj)
                
            #Check for max accelerations
            fac = np.max(np.max(np.abs(qddoti), axis=0)/qddot_max)
            if fac>1:
                rospy.loginfo("JMove extending time due to acceleraton qddot_max by factor {}".format(fac))
                t = time[-1] * fac
                time=np.arange(self.tsamp,t,self.tsamp)
                qi, qdoti, qddoti = jtraj(q0, q, time, traj=traj)
                
            # Record the trajectory
            self.qi = qi
            self.qdoti = qdoti
            if len(self.recorded_jmove_trajectories) > 5:self.recorded_jmove_trajectories= self.recorded_jmove_trajectories[1:]
            self.recorded_jmove_trajectories.append([qi, qdoti, self._command_int, self._actual_int])
            
            for qt, qdt in zip(qi,qdoti):
                if self._do_motion_check and self._motion_check_callback is not None:
                    tmperr = self._motion_check_callback(self)
                    if tmperr>0:
                        self.WarningMessage('Motion aborted')
                        return tmperr
                    
                # Timer nastavi na nulo. Potem tukej glej ali je ta dt res tak kot tsamp. Ce je premajhen ga pocakas. Nastavis last command time in gledas razlike.
                self.GoTo_q(q = qt, qdot = qdt, qddot = None, trq = trq, wait = self.tsamp)
            self.GoTo_q(q = qt, qdot = np.zeros(self.nj),qddot = None, trq = trq, wait = wait, last_move = True)
            
        self.Stop()
        self.Message('JMove finished', 2)
        return tmperr

    def JLine(self, q, t, state='Commanded', trq=None, wait=None):
        self.Message('JLine -> JMove', 2)
        tmperr = self.JMove(q, t, trq=trq, traj='Trap', wait=wait)
        return tmperr

    #def JMoveFor(self, dq, t, state='Commanded', trq=None, traj='poly', wait=None):
    def JMoveFor(self, dq, t, state='Actual', trq=None, traj='poly', wait=None):
        self.safety_check()
        
        dq = self.jointvar(dq)
        q0 = self.GetJointPos(state=state)
        q = q0+dq
        self.Message('JMoveFor -> JMove', 2)
        tmperr = self.JMove(q, t, trq=trq, traj=traj, wait=wait)
        return tmperr

    def GoTo_T(self, T, v, FT, wait=None, last_move = False, **kwargs):
        """ %GoTo_T Move to task pose T
            %
            % Inputs:
            %   T     task position
            %   v     task velocity (6 x 1)
            %   FT    task force/torques (6 x 1)
            %
            % Options:
            %   Coordinate system: 'TaskSpace' = {'World','Robot','Object','Tool'}
            %   Wait    maximal wait time in final position - default: 10*tsamp
            %
            % Options for kinematic controller:
            %   Null-space optimization: 'NullSpaceTask' = {'None','Manipulability','JointLimits','ConfOptimization'}
            %   TaskDOF selection of DOF in task space (vector 6 x 1 of {0,1})
            %   Kp      task space position gain
            %   Kns     null-space gain
            %   qopt    optimal pose configuration
        """
                
        if 'task_space' not in kwargs.keys():
            taskspace = self.Default_TaskSpace
        else:
            taskspace = kwargs['task_space']
        
        tmperr = 0
        
        if self._control_strategy in ['CartesianImpedance', 'CartesianPose', 'CartesianVelocity']:
            if taskspace in ['World', 'Robot']:
                self._command_int.p = T[0:3,3]
                self._command_int.R = T[0:3, 0:3]
                self._command_int.v = v
                self._command_int.FT = FT
                
                self._command_int.x = T[0:3,-1]
            elif taskspace == 'Object':
                T=self.ObjectToWorld(T)
                v=self.ObjectToWorld(v)
                FT=self.ObjectToWorld(FT)
                            
                self._command_int.p=T[0:3,3]
                self._command_int.R=T[0:3, 0:3]
                self._command_int.v=v
                self._command_int.FT=FT
            else:
                rospy.loginfo("GoTo_T : Unsupported coordinate system")
            tmperr = self.GoTo_X(t2x(T), v, FT, max(wait, self.tsamp), **kwargs)
            self.GetState()
            return tmperr
        
        elif self._control_strategy in ['JointPosition', 'JointVelocity', 'JointImpedance', 'JointPositionTrajectory']:
            tmperr = self.GoTo_TC(T, v, FT, last_move = last_move, **kwargs)
            return tmperr
        else:
            rospy.loginfo('GoTo_T : Control strategy not supported!')

    # def GoTo_TCx(self, x, v, FT, wait=None, task_space='World', pos_err=None, ori_err=None, \
    #             task_cont_space='Robot', taskDOF=(1,1,1,1,1,1), null_space_task='JointLimits', \
    #             q_opt=None, x_opt=None, v_ns=None, Kp=50, Kns=50):
    #     return 0
    def GoTo_TC(self, x, v, FT, wait=None, last_move = False, **kwargs):
        
        tx = self.simtime()
        rx = x2x(x)
        if v is None:
            v = np.zeros(6)
        else:
            v = vector(v, dim=6)
        if FT is None:
            FT = np.zeros(6)
        else:
            FT = vector(FT, dim=6)
        if wait is None:
            wait = self.tsamp
        kwargs.setdefault('task_space','World')
        kwargs.setdefault('pos_err',0.002)
        kwargs.setdefault('ori_err',0.02)
        kwargs.setdefault('task_cont_space','Robot')
        kwargs.setdefault('task_DOF',np.ones(6))
        kwargs.setdefault('null_space_task','None')
        kwargs.setdefault('Kp',200)
        kwargs.setdefault('Kns',100)

        task_DOF = vector(kwargs['task_DOF'], dim=6)
        Sind = np.where(task_DOF>0)[0]
        Kp = kwargs['Kp']*self.tsamp
        Kns = kwargs['Kns']*self.tsamp
        uNS = np.zeros(self.nj)

        if check_option(kwargs['task_space'], 'World'):
            self._command_int.x = rx
            self._command_int.v = v
            self._command_int.FT = FT
            rx = self.WorldToBase(rx)
            v = self.WorldToBase(v)
            FT = self.WorldToBase(FT)
        elif check_option(kwargs['task_space'], 'Robot'):
            self._command_int.x = self.BaseToWorld(rx)
            self._command_int.v = self.BaseToWorld(v)
            self._command_int.FT = self.BaseToWorld(FT)
        elif check_option(kwargs['task_space'], 'Object'):
            rx = self.ObjectToWorld(rx)
            v = self.ObjectToWorld(v)
            FT = self.ObjectToWorld(FT)
            self._command_int.x = rx
            self._command_int.v = v
            self._command_int.FT = FT
            rx = self.WorldToBase(rx)
            v = self.WorldToBase(v)
            FT = self.WorldToBase(FT)
        else:
            raise ValueError(f'Task space {kwargs["task_space"]} not supported')

        imode = self._command_int.mode
        if check_option(kwargs['null_space_task'], 'None'):
            self._command_int.mode = 2.1
        elif check_option(kwargs['null_space_task'], 'Manipulability'):
            self._command_int.mode = 2.2
        elif check_option(kwargs['null_space_task'], 'JointLimits'):
            self._command_int.mode = 2.3
            q_opt = (self.q_max+self.q_min)/2
        elif check_option(kwargs['null_space_task'], 'ConfOptimization'):
            self._command_int.mode = 2.4
            kwargs.setdefault('q_opt',self.q_home)
            q_opt = vector(kwargs['q_opt'], dim=self.nj)
        elif check_option(kwargs['null_space_task'], 'PoseOptimization'):
            self._command_int.mode = 2.5
            km = self.kinmodel(self.q_home)
            kwargs.setdefault('x_opt',km[0])
            x_opt = x2x(kwargs['x_opt'])
            if check_option(kwargs['task_space'], 'World'):
                x_opt = self.WorldToBase(x_opt)
            elif check_option(kwargs['task_space'], 'Object'):
                x_opt = self.ObjectToWorld(x_opt)
                x_opt = self.WorldToBase(x_opt)
        elif check_option(kwargs['null_space_task'], 'TaskVelocity'):
            self._command_int.mode = 2.6
            kwargs.setdefault('v_ns',np.zeros(6))
            rv = vector(kwargs['v_ns'], dim=6)
            if check_option(kwargs['task_space'], 'World'):
                rv = self.WorldToBase(rv)
            elif check_option(kwargs['task_space'], 'Object'):
                rv = self.ObjectToWorld(rv)
                rv = self.WorldToBase(rv)
        elif check_option(kwargs['null_space_task'], 'JointVelocity'):
            self._command_int.mode = 2.7
            kwargs.setdefault('qdot_ns',np.zeros(self.nj))
            rqdn = vector(kwargs['qdot_ns'], dim=self.nj)
        else:
            raise ValueError(f'Null-space task {kwargs["null_space_task"]} not supported')
        
        tmperr = 0
        rp = copy.deepcopy(rx[:3])
        rR = copy.deepcopy(q2r(rx[3:]))
        
        self._command_int.p = rp
        self._command_int.R = rR
        self._command_int.v = v

        while True:
            # This commanded qq below is not set anywhere (during CMove) so we set it to actual
            #self.GetState()
            #self._command_int.q = self._actual_int.q
            
            qq = self._command_int.q
            #np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
            p, R, J = self.kinmodel(q = qq, out='pR')
            ep = rp-p
            eR = qerr(r2q(rR@R.T))
            ee = np.hstack((ep, eR))

            if check_option(kwargs['task_cont_space'], 'World'):
                RC = np.kron(np.eye(2),self.TBase[:3,:3]).T
            elif check_option(kwargs['task_cont_space'], 'Robot'):
                RC = np.eye(6)
            elif check_option(kwargs['task_cont_space'], 'Tool'):
                RC = np.kron(np.eye(2),R).T
            elif check_option(kwargs['task_cont_space'], 'Object'):
                RC = np.kron(np.eye(2),self.TObject[:3,:3]).T
            else:
                raise ValueError(f'Task space {kwargs["task_cont_space"]} not supported')

            ee = RC@ee
            J = RC@J
            v = RC@v
            ux = v+Kp*ee
            trq = J.T@FT
            ux = ux[Sind]
            JJ = J[Sind,:]
            Jp = np.linalg.pinv(JJ)
            NS = np.eye(self.nj)-Jp@JJ

            if check_option(kwargs['null_space_task'], 'None'):
                qdn = np.zeros(self.nj)
            elif check_option(kwargs['null_space_task'], 'Manipulability'):
                fun = lambda q: self.manipulability(q, task_space=kwargs['task_space'], task_DOF=kwargs['task_DOF'])
                qdotn = grad(fun, qq)
                qdn = Kns*qdotn
            elif check_option(kwargs['null_space_task'], 'JointLimits'):
                qdn = Kns*(q_opt-qq)
            elif check_option(kwargs['null_space_task'], 'ConfOptimization'):
                qdn = Kns*(q_opt-qq)
            elif check_option(kwargs['null_space_task'], 'PoseOptimization'):
                een = xerr(x_opt,map_pose(p=p, R=R))
                qdn = Kns*np.linalg.pinv(J)@een
            elif check_option(kwargs['null_space_task'], 'TaskVelocity'):
                qdn = np.linalg.pinv(J)@rv
            elif check_option(kwargs['null_space_task'], 'JointVelocity'):
                qdn = rqdn

            uNS = NS@qdn
            u = Jp@ux+uNS
            rqd = Jp@v[Sind]+uNS
            
            #rqd = np.clip(rqd, -self.qdot_max, self.qdot_max) # NA NOVO DODANO - pred tem rqd ni bil clippan sploh
            
            #u = np.clip(u, -self.qdot_max, self.qdot_max)

            q_velocities_factor = np.abs(rqd) / (self.qdot_max*0.3)
            max_q_velocity_factor = np.max(q_velocities_factor)
            
            # If this factor were to be smaller than 1, the trajectory would be sped up.
            if max_q_velocity_factor < 1:
                max_q_velocity_factor = 1
            rqd = rqd / max_q_velocity_factor
            new_tsamp = self.tsamp * max_q_velocity_factor

            rq = qq+u*self.tsamp

            if self.CheckJointLimits(rq):
                self._command_int.mode = imode
                self._command_int.qdot = np.zeros(self.nj)
                self._command_int.v = np.zeros(6)
                if tm() - self.last_warning_print_time > 3:
                    self.WarningMessage('Joint limits reached: ' + str(self.q) )
                    self.last_warning_print_time = tm()
                return 99

            self.GoTo_q(q = rq, qdot = rqd, qddot = None, trq = trq, wait = new_tsamp, last_move = last_move)
            # self.Update()

            if self.simtime()-tx>wait or (np.linalg.norm(ep)<kwargs['pos_err'] and np.linalg.norm(eR)<kwargs['ori_err']):
                self._command_int.mode = imode
                return 0

    def CMove(self, x, t, traj='poly', wait=None, FT=None, **kwargs):
        
        self.safety_check()

        x = self.spatial(x)
        if wait is None:
            wait = self.tsamp
        if FT is None:
            FT = np.zeros(6)
        else:
            FT = vector(FT, dim=6)
            
        kwargs.setdefault('v_max_factor', 1) # If not specified, maximal robot accelerations are allowed
        kwargs.setdefault('a_max_factor', 1)
        
        assert 0 < kwargs['v_max_factor'] <= 1
        assert 0 < kwargs['a_max_factor'] <= 1
        
        v_max = self.v_max * kwargs['v_max_factor']
        a_max = self.a_max * kwargs['a_max_factor']
        
        #print("vmax shape:", v_max.shape)
        #assert v_max.shape == (7,1)
            
        kwargs.setdefault('Kns', 100) # Here we need to set Kns, for SHAPING ! Otherwise, null space correction will cause boundary abs(v) > 0.

        kwargs.setdefault('short',True)
        kwargs.setdefault('kinematics','Calculated')
        #kwargs.setdefault('state','Actual')  # Replaced commanded with Actual !  TODO - BUG here - commanded and actual are not the same. now we use Actual
        kwargs.setdefault('state', 'Commanded')

        # BUG : "Commanded" arg makes GetPose calculate cartesian position using _command_int.q. However, if using CartesianImpedance controller, _command_int.q values are not updated.
        
        kwargs.setdefault('task_space', 'World')
        kwargs.setdefault('changing_stiffness', False )
        
        #if self._control_strategy in ['JointPositionTrajectory']:
        #    self.GetState()
        #    self._command_int.q = copy.deepcopy(self._actual_int.q)
        
        if self._control_strategy not in ['CartesianImpedance']:
            kwargs['changing_stiffness']  = False
        
        if check_option(kwargs['task_space'], 'Tool'):
            task_space = 'World'
            kwargs['task_space'] = 'World'
            
            T0 = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            if x.shape==(4, 4):
                rT = T0@x
            elif isvector(x, dim=7):
                rT = T0@x2t(x)
            elif x.shape==(3, 3):
                rT = T0@map_pose(R=x, out='T')
            elif isvector(x, dim=3):
                rT = T0@map_pose(p=x, out='T')
            elif isvector(x, dim=4):
                rT = T0@map_pose(Q=x, out='T')
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            
            if x.shape==(4, 4):
                rT = x
            elif isvector(x, dim=7):
                rT = x2t(x)
            elif x.shape==(3, 3):
                p0 = self.GetPos(state= kwargs['state'], task_space=kwargs['task_space'], kinematics=kwargs['kinematics'])
                rT = map_pose(R=x, p=p0, out='T')
            elif isvector(x, dim=4):
                p0 = self.GetPos(state= kwargs['state'], task_space=kwargs['task_space'], kinematics=kwargs['kinematics'])
                rT = map_pose(Q=x, p=p0, out='T')
            elif isvector(x, dim=3):
                R0 = self.GetOri(state= kwargs['state'], out='R', task_space=kwargs['task_space'], kinematics=kwargs['kinematics'])
                rT = map_pose(p=x, R=R0, out='T')
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        
        rx = t2x(rT)
                
        self.Message('CMove started', 2)
        self._command_int.mode = 2
        
        #rospy.loginfo("CMove init: {}".format(self._command_int.x[0:3]))
        #rospy.loginfo("CMove finish: {}".format(rx[0:3]))
        #d = self._command_int.x[0:3] - rx[0:3]
        #rospy.loginfo("CMove d: {}".format(d))
        #rospy.loginfo("Init x: {}".format(self._actual_int.x))

        self.Start()
        
        #tmp_buffer = []
        if self._control_strategy in ['CartesianImpedance']:
            # CartImp uses GoTo_X which doesn't set the self._command_int.q value after doing a move (Even if you add it there, due to some unexplained bugs it still won't work)
            self._command_int.q = copy.deepcopy(self.state.q)
            #print("CMOVE set command q")
        elif self._control_strategy in ['JointPositionTrajectory']:
            #print("HIT2")
            0
        else:
            print("Invalid control strategy for CMove!")
        
        #xe = np.amax(np.abs(self.TaskDistance(x))/self.v_max)
        #if xe<0.02:
        #    self.Message('CMove close to target; not doing the move.',2)
        #    return 0
        
        tmperr = 0
        if t<=2*self.tsamp:
            self.GoTo_T(x2t(rx), np.zeros(6), FT, wait=wait, **kwargs)
        else:
            time=np.arange(self.tsamp,t,self.tsamp)
            #print(kwargs['state'], kwargs['kinematics'])
            x0 = self.GetPose(**kwargs)
            rospy.loginfo("x0: {}".format(x0))
            #rospy.loginfo("q0: {}".format(self._command_int.q))

            xi, vi, ai = ctraj(x0, rx, time, traj=traj, short=kwargs['short'])
            
            rospy.loginfo("rx: {}".format(rx))
            
            # Check for max velocity
            fac = np.max(np.max(np.abs(vi), axis=0)/v_max)
            if fac>1:
                t = time[-1] * fac
                rospy.loginfo("CMove extending time due to vmax by factor {}".format(fac))
                time=np.arange(self.tsamp,t,self.tsamp)
                xi, vi, ai = ctraj(x0, rx, time, traj=traj, short=kwargs['short'])
                
            # Check for max acceleration
            fac = np.max(np.max(np.abs(ai), axis=0)/a_max)
            if fac>1:
                rospy.loginfo("CMove extending time due to amax by factor {}".format(fac))
                t = time[-1] * fac
                time=np.arange(self.tsamp,t,self.tsamp)
                xi, vi, ai = ctraj(x0, rx, time, traj=traj, short=kwargs['short'])
                
            self.xi = xi
            self.vi = vi
            # Record the trajectory
            if len(self.recorded_cmove_trajectories) > 5:self.recorded_cmove_trajectories= self.recorded_cmove_trajectories[1:]
            self.recorded_cmove_trajectories.append([xi, vi, self._command_int, self._actual_int])
                
            # Hack for null space stiffness (even when using joint trajectory controller. During the beginning of movement, null space must be ignored so the velocity boundary conds. are 0.
            tm = np.linspace(-np.pi/2, np.pi*3/2, xi.shape[0])
            Kns = kwargs['Kns'] * (np.sin(tm) + 1)
            
            if kwargs['changing_stiffness']:
                #print('ch')
                # this will gradually scale the stiffness so it achives max stiffness in the last step
                #shaping=np.linspace(0.5,1,xi.shape[0])
                x = np.linspace(-20, 30, xi.shape[0])
                shaping =  0.6 + 0.4 * (np.tanh(0.2*x)+1) / 2
                #shaping= 0.6 + 0.4 * (np.tanh(0.3*x)+1) / 2
                Kps = [s*self.cartesian_compliance.Kp for s in shaping]
                Krs = [s*self.cartesian_compliance.Kr for s in shaping]
                #Ds = [s*self.cartesian_compliance.D for s in shaping]
                                
            #print("Len of XI: %d"%len(xi))
            for i, (xt, vt) in enumerate(zip(xi,vi)):
                if self._do_motion_check and self._motion_check_callback is not None:
                    tmperr = self._motion_check_callback(self)
                    if tmperr>0:
                        self._command_int.qdot = np.zeros(self.nj)
                        self._command_int.v = np.zeros(6)
                        self.WarningMessage('Motion check stopped motion')
                        return tmperr
                
                if kwargs['changing_stiffness']:
                    tmperr = self.GoTo_T(x2t(xt), vt, FT, wait=wait, Kp=Kps[i], Kr=Krs[i])
                else:
                    kwargs['Kns'] = Kns[i]
                    tmperr = self.GoTo_T(x2t(xt), vt, FT, wait=wait, **kwargs)

                if tmperr>0:
                    self.WarningMessage('Motion aborted')
                    return tmperr
                
            tmperr=self.GoTo_T(x2t(rx), np.zeros(6), FT, wait=wait, last_move = True, **kwargs)
        
        self.Stop() # Untested and left commented out.
        self.Message('CMove finished', 2)
        
        # Hack
        #sleep(0.5)
        self.GetState()
        #self._actual_int.q =  copy.deepcopy(self.state.q)
        #self._command_int.x = x2t(rx)
        #self.ResetCurrentTarget(send_msg = False)
        
        return tmperr

    def CMoveFor(self, dx, t, traj='poly', wait=None, task_space='World', FT=None, **kwargs):
        self.safety_check()

        dx = self.spatial(dx)
        kwargs.setdefault('kinematics','Calculated')
        kwargs.setdefault('state','Commanded')
        #kwargs.setdefault('state', 'Actual')
        if check_option(task_space, 'Tool'):
            task_space = 'World'
            T0 = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            if isvector(dx, dim=3):
                rT = T0@map_pose(p=dx, out='T')
            elif dx.shape==(3, 3):
                rT = T0@map_pose(R=dx, out='T')
            elif isvector(dx, dim=4):
                rT = T0@map_pose(Q=dx, out='T')
            else:
                raise ValueError(f'Parameter shape {dx.shape} not supported')
        else:
            rT = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            if isvector(dx, dim=3):
                rT[:3,3] += dx
            elif dx.shape==(3, 3):
                rT[:3,:3] = dx@rT[:3,:3] 
            elif isvector(dx, dim=4):
                rT[:3,:3] = q2r(dx)@rT[:3,:3] 
            else:
                raise ValueError(f'Parameter shape {dx.shape} not supported')
        rx = t2x(rT)
        
        #print("CMoveFor:rx:")
        #print(rx)
        #rospy.loginfo("Current position:")
        #rospy.loginfo(self.x)
        self.Message('CMoveFor -> CMove', 2)
        #rospy.loginfo("CMoveFor moving to")
        #rospy.loginfo(rx)
        tmperr = self.CMove(rx, t, traj=traj, wait=wait, task_space=task_space, FT=FT, **kwargs)
        
        return tmperr

    def CApproach(self, x, dx, t, traj='poly', wait=None, task_space='World', FT=None, **kwargs):
        x = self.spatial(x)
        dx = vector(dx, dim=3)
        if x.shape==(4, 4):
            rx = map_pose(T=x)
        elif isvector(x, dim=7):
            rx = x
        else:
            raise ValueError(f'Parameter shape {x.shape} not supported')
        rx[:3] += dx 
        self.Message('CApproach -> CMove', 2)
        tmperr = self.CMove(rx, t, traj=traj, wait=wait, task_space=task_space, FT=FT, **kwargs)
        return tmperr

    def CLine(self, x, t, wait=None, task_space='World', FT=None, **kwargs):
        self.Message('CLine -> CMove', 2)
        tmperr = self.CMove(x, t, traj='Trap', wait=wait, task_space=task_space, FT=FT, **kwargs)
        return tmperr

    def CLineFor(self, dx, t, wait=None, task_space='World', FT=None, **kwargs):
        dx = self.spatial(dx)
        kwargs.setdefault('kinematics','Calculated')
        kwargs.setdefault('state','Commanded')
        if check_option(task_space, 'Tool'):
            task_space = 'World'
            T0 = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            if isvector(dx, dim=3):
                rT = T0@map_pose(p=dx, out='T')
            elif dx.shape==(3, 3):
                rT = T0@map_pose(R=dx, out='T')
            elif isvector(dx, dim=4):
                rT = T0@map_pose(Q=dx, out='T')
            else:
                raise ValueError(f'Parameter shape {dx.shape} not supported')
        else:
            rT = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            if isvector(dx, dim=3):
                rT[:3,3] += dx
            elif dx.shape==(3, 3):
                rT[:3,:3] = dx@rT[:3,:3] 
            elif isvector(dx, dim=4):
                rT[:3,:3] = q2r(dx)@rT[:3,:3] 
            else:
                raise ValueError(f'Parameter shape {dx.shape} not supported')
        rx = t2x(rT)
        self.Message('CLineFor -> CMove', 2)
        tmperr = self.CMove(rx, t, traj='Trap', wait=wait, task_space=task_space, FT=FT, **kwargs)
        return tmperr

    def CArc(self, x, pC, t, traj='poly', wait=None, task_space='World', FT=None, **kwargs):
        x = self.spatial(x)
        pC = vector(pC, dim=3)
        if wait is None:
            wait = self.tsamp
        if FT is None:
            FT = np.zeros(6)
        else:
            FT = vector(FT, dim=6)

        kwargs.setdefault('short',True)
        kwargs.setdefault('kinematics','Calculated')
        kwargs.setdefault('state','Commanded')
        if check_option(task_space, 'Tool'):
            task_space = 'World'
            T0 = self.GetPose(out='T', task_space=task_space, kinematics=kwargs['kinematics'])
            rpC = T0[:,3,:3]@pC
            if x.shape==(4, 4):
                rT = T0@x
            elif isvector(x, dim=7):
                rT = T0@x2t(x)
            elif x.shape==(3, 3):
                rT = T0@map_pose(R=x, out='T')
            elif isvector(x, dim=3):
                rT = T0@map_pose(p=x, out='T')
            elif isvector(x, dim=4):
                rT = T0@map_pose(Q=x, out='T')
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            rpC = pC
            if x.shape==(4, 4):
                rT = x
            elif isvector(x, dim=7):
                rT = x2t(x)
            elif x.shape==(3, 3):
                p0 = self.GetPos(state='Commanded', task_space=task_space, kinematics=kwargs['kinematics'])
                rT = map_pose(R=x, p=p0, out='T')
            elif isvector(x, dim=4):
                p0 = self.GetPos(state='Commanded', task_space=task_space, kinematics=kwargs['kinematics'])
                rT = map_pose(Q=x, p=p0, out='T')
            elif isvector(x, dim=3):
                R0 = self.GetOri(state='Commanded', out='R', task_space=task_space, kinematics=kwargs['kinematics'])
                rT = map_pose(p=x, R=R0, out='T')
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        rx = t2x(rT)
        self.Message('CArc started', 2)
        self._command_int.mode = 2
        self.Start
        tmperr = 0
        if t<=2*self.tsamp:
            self.GoTo_T(x2t(rx), np.zeros(6), FT, wait=wait, task_space=task_space, **kwargs)
        else:
            time=np.arange(self.tsamp,t,self.tsamp)
            x0 = self.GetPose(task_space=task_space, **kwargs)
            xi, vi, _ = carctraj(x0, rx, rpC, time, traj=traj, short=kwargs['short'])
            fac = np.max(np.max(np.abs(vi), axis=0)/self.v_max)
            if fac>1:
                time=np.arange(self.tsamp,t*fac,self.tsamp)
                xi, vi, _ = carctraj(x0, rx, rpC, time, traj=traj, short=kwargs['short'])
            for xt, vt in zip(xi,vi):
                if self._do_motion_check and self._motion_check_callback is not None:
                    tmperr = self._motion_check_callback(self)
                    if tmperr>0:
                        self._command_int.qdot = np.zeros(self.nj)
                        self._command_int.v = np.zeros(6)
                        self.WarningMessage('Motion check stopped motion')
                        return tmperr
                tmperr = self.GoTo_T(x2t(xt), vt, FT, wait=0, task_space=task_space, **kwargs)
                if tmperr>0:
                    self.WarningMessage('Motion aborted')
                    return tmperr

            tmperr=self.GoTo_T(x2t(rx), np.zeros(6), FT, wait=wait, task_space=task_space, **kwargs)
        self.Stop
        self.Message('CArc finished', 2)
        
        return tmperr
    
    def JPath_new(self, path, t):
        #raise Exception("Not working yet")
        """ Min jerk traj in joint space"""
        assert self._control_strategy == 'JointPositionTrajectory' # Not working for other controllers yet
        
        self.safety_check()
        
        qs = np.array(path)
        if 1 :
            # Make a min jerk trajectory in joint space, given these vias to make it aprox cart.
            new_size = 5
            
            qs_trajectory, psg = generate_min_jerk(qs, t, self.tsamp, new_size, control_space_for_min_jerk = 'q', return_values = 'q')
            #print("qs.shape", qs_trajectory.shape)
        
            self.orig_traj = qs_trajectory
            
            s = np.linspace(0,t,qs_trajectory.shape[0])
            
            time=np.arange(0,t+self.tsamp,self.tsamp)
            
            tcks = []
            qis = []
            qdots = []
            qddots = []
            #print("poly order 5", qs.shape)
            for i in range(0,7):
                #tck = CubicSpline(s, qs_trajectory[:,i], bc_type = 'natural')
                tck = make_interp_spline(s, qs_trajectory[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
                #tck = make_interp_spline(s, qs_trajectory[:,i], k=4, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0)]))
                #tck = make_interp_spline(s, qs_trajectory[:,i], k=5, bc_type=([(2, 0.0), (3, 0.0)], [(2, 0.0), (3, 0.0)]))
                
                qi = splev(time,tck)
                
                qdoti_bspline = tck.derivative()
                qdoti = splev(time, qdoti_bspline)
                
                qddoti_bspline = qdoti_bspline.derivative()
                qddoti = splev(time, qddoti_bspline)
                
                qis.append(qi)
                qdots.append(qdoti)
                qddots.append(qddoti)
                
            qi = np.vstack(qis)
            qi = np.transpose(qi)
            qs = qi
            
            #tmp = np.transpose(qs);print("qdot shape", tmp.shape, s.shape)
            #qdot = gradientPath(path = qs, s = time)
            #print("qdot shape", qdot.shape)

            qdot = np.vstack(qdots)
            qdot = np.transpose(qdot)
            
            qddot = np.vstack(qddots)
            qddot = np.transpose(qddot)
            
            sh = qs.shape[0]
            z = np.zeros(7)
            
            ts = np.linspace(0, t, qs.shape[0])
        
            dt = ts[1] - ts[0]
            
            # Get the joint velocities
            #qdot = gradientPath(path = qs, s = ts)
        
        self.qq =qs
        self.vv = qdot
        self.aa = qddot
        
        # Check trajectory (debug)
        # Check joint limits in qs
        # Check max velocities in qdot
        # Check max accelerations in qddot
        
        self.Start()
        zeros = np.zeros(7)
        
        for i in range(qs.shape[0]-1):
            #dt = ts[i+1] - ts[i]
            self.GoTo_q(q = qs[i, :], qdot = qdot[i,:], qddot = qddot[i,:], trq = zeros, wait =dt, do_not_publish_msg=False, last_move =False)
            #self.GoTo_q(q = qs[i, :], qdot =  qdot[i,:], qddot = zeros, trq = z, wait =dt, do_not_publish_msg=False, last_move =False)

        self.GoTo_q(q = qs[-1, :], qdot = qdot[-1,:], qddot = qddot[-1,:], trq = zeros, wait =dt, do_not_publish_msg=False, last_move =True)
        #self.GoTo_q(q = qs[-1, :], qdot =  qdot[-1,:], qddot = zeros, trq = z, wait =dt, do_not_publish_msg=False, last_move =True)
        return 0

    def CPath_new(self, path, t):
        """ Testing the new min jerk traj generator"""
        # FAST MOVE MIN JERK Q GENERATION
        
        assert self._control_strategy == 'JointPositionTrajectory' # Not working for other controllers yet
        # Interpolate the cart path to a reasonable amount (max 200 pts)
        start_calc = tm()

        # Calculate the inv kin.
        self.safety_check()

        qs = []
        self.GetState()
        if self._control_strategy == 'JointPositionTrajectory':
            a = np.array(self._actual_int.q)
            b = np.array(self._command_int.q)
            q_error = np.abs(a-b)
            q_error_sum = np.sum(q_error)
            reset = 0
            if q_error_sum > 0.02:
                reset = 1
                self._command_int.q = copy.deepcopy(self._actual_int.q)
        
        init_q = copy.deepcopy(self._command_int.q)
        qs.append(init_q)
        for i in range(1, path.shape[0]):
            q = self.x2q_invkin(x = path[i, :], q_last = init_q, return_trajectory = False)
            init_q = copy.deepcopy(q)

            qs.append(q)

        qs = np.array(qs)
        
        dtt =  tm() - start_calc
        rospy.loginfo("Cpath_new v2 invkin took {} s".format(dtt))
        
        if 1 :
    
            # Make a min jerk trajectory in joint space, given these vias to make it aprox cart.
            new_size = 5
            
            qs_trajectory, psg = generate_min_jerk(qs, t, self.tsamp, new_size, control_space_for_min_jerk = 'q', return_values = 'q')
            #print("qs.shape", qs_trajectory.shape)
        
            self.orig_traj = qs_trajectory
            
            s = np.linspace(0,t,qs_trajectory.shape[0])
            #s, _, _ = jpoly(q0 = [0], q1 = [t], t=np.linspace(0,t,qs_trajectory.shape[0]))

            time=np.arange(0,t+self.tsamp,self.tsamp)
            
            tcks = []
            qis = []
            qdots = []
            qddots = []
            #print("poly order 5", qs.shape)
            for i in range(0,7):
                
                #print(qs_trajectory[0:7, i])
                #tck = CubicSpline(s, qs_trajectory[:,i], bc_type = 'natural')
                tck = make_interp_spline(s, qs_trajectory[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
                #tck = make_interp_spline(s, qs_trajectory[:,i], k=4, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0)]))
                #tck = make_interp_spline(s, qs_trajectory[:,i], k=5, bc_type=([(2, 0.0), (3, 0.0)], [(2, 0.0), (3, 0.0)]))
                
                qi = splev(time,tck)
                
                qdoti_bspline = tck.derivative()
                qdoti = splev(time, qdoti_bspline)
                
                qddoti_bspline = qdoti_bspline.derivative()
                qddoti = splev(time, qddoti_bspline)
                
                qis.append(qi)
                qdots.append(qdoti)
                qddots.append(qddoti)
                
            qi = np.vstack(qis)
            qi = np.transpose(qi)
            qs = qi
            
            #tmp = np.transpose(qs);print("qdot shape", tmp.shape, s.shape)
            #qdot = gradientPath(path = qs, s = time)
            #print("qdot shape", qdot.shape)

            qdot = np.vstack(qdots)
            qdot = np.transpose(qdot)
            
            qddot = np.vstack(qddots)
            qddot = np.transpose(qddot)
            
            #print("qs shape:", qs.shape)
            #print("qdot shape", qdot.shape)
            ###
            ### UPORABA ORIGINAL TRAJEKTORIJE IZ min_jerk_generatorja
            #print("Cpath new v4")
            #qs = qs_trajectory
            #qdot = np.gradient(qs, axis=0)
            #qddot = np.gradient(qdot, axis=0)
            ###
            
            sh = qs.shape[0]
            z = np.zeros(7)
            
            ts = np.linspace(0, t, qs.shape[0])
            
            #n_pts = sh
            #ss = np.linspace(0, np.pi, n_pts)
            #sine = np.sin(ss)
            #ts = np.cumsum(sine)
            #ts = ts/ts[-1]
            #ts = ts * t 
        
            dt = ts[1] - ts[0]
            
            # Get the joint velocities
            #qdot = gradientPath(path = qs, s = ts)
            a,b,c = qs, qdot, qddot
            
        else:
            0
            """ Koda naj bi bla v redu ampak se robot trese med izvajanjem
            time=np.arange(self.tsamp,t,self.tsamp)
            t_tmp = np.linspace(0,t,qs.shape[0])
            s, _, _ = jpoly(q0 = [0], q1 = [1], t=t_tmp)
            
            qi = []
            qdots = []
            qddots = []

            for i in range(0,7):
                tck = make_interp_spline(s, qs[:,i], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]))
                qi.append(tck(time/time[-1]))
                
                vel = tck.derivative()
                qdots.append(vel(time/time[-1]))
                
                acc = vel.derivative()
                qddots.append(acc(time/time[-1]))
                
            qs = np.vstack(qi).transpose()
            qdot = np.vstack(qdots).transpose()
            
            qddot = np.vstack(qddots).transpose()
            dt = time[1] - time[0]
            d,e,f = qs,qdot, qddot
            """
         
        #return a,b,c,d,e,f
        #return qs, qdot, qddot, dt
            
        end_time = tm()
        time_to_calc = end_time-start_calc
        rospy.loginfo("CPath_new FULL calc took t={}s".format(time_to_calc))
        
        self.qq =qs
        self.vv = qdot
        self.aa = qddot
        
        # Check trajectory (debug)
        # Check joint limits in qs
        # Check max velocities in qdot
        # Check max accelerations in qddot
        
        self.Start()
        zeros = np.zeros(7)
        
        for i in range(qs.shape[0]-1):
            #dt = ts[i+1] - ts[i]
            self.GoTo_q(q = qs[i, :], qdot = qdot[i,:], qddot = qddot[i,:], trq = zeros, wait =dt, do_not_publish_msg=False, last_move =False)
            #self.GoTo_q(q = qs[i, :], qdot =  qdot[i,:], qddot = zeros, trq = z, wait =dt, do_not_publish_msg=False, last_move =False)

        self.GoTo_q(q = qs[-1, :], qdot = qdot[-1,:], qddot = qddot[-1,:], trq = zeros, wait =dt, do_not_publish_msg=False, last_move =True)
        #self.GoTo_q(q = qs[-1, :], qdot =  qdot[-1,:], qddot = zeros, trq = z, wait =dt, do_not_publish_msg=False, last_move =True)
        return 0

    def CPath(self, path, t, wait=None, task_space='World', FT=None, only_return_path = False, **kwargs):
        #rospy.loginfo("CPath is not working!")
        #rospy.loginfo("CPath calculated velocity boundary values are not 0 !")

        #return 0
        raise Exception("CPath does not work currently!")
        
        assert ismatrix(path, shape=7)
        if wait is None:
            wait = self.tsamp
        if FT is None:
            FT = np.zeros(6)
        else:
            FT = vector(FT, dim=6)

        self.Message('CPath started', 2)
        self._command_int.mode = 2
        
        N = path.shape[0]
        s = np.linspace(0,t,N)
        time=np.arange(0,t+self.tsamp,self.tsamp)
        """
        # Old
        #xi = interpCartesianPath(s, path, time)
        """
        
        # New using scipy higher order interpolation
        tcks = []
        xis = []
       
        k = min(5, path.shape[0]-1)
        for i in range(0,3):
            # Cartesian pos
            tck = make_interp_spline(x = s, y = path[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
            tcks.append(tck)
            xi = splev(time,tck)
            xis.append(xi)
                
        #### NEW a 
        """
        n_slerp_interp_pts = 3 

        interp_quats = []
        for i in range(0, path.shape[0] -1):
            s = np.linspace(0,1, n_slerp_interp_pts)
            sl = slerp(Q1 = path[i,3:], Q2 = path[i+1, 3:], s = s, short = True)
            #print(sl)
            interp_quats.append(sl)
        interp_quats = np.vstack(interp_quats)
        interp_quats = uniqueQuaternion(interp_quats)

        qs = []
        tcks = []
        s = np.linspace(0, path.shape[0], (path.shape[0]-1)*n_slerp_interp_pts)
        """
        qs = []
        tcks = []
        for i in range(3,7):
        #for i in range(0,4):
            tck = make_interp_spline(x = s, y = path[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
            #tck = make_interp_spline(x = s, y = interp_quats[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
            qi = splev(time,tck)
            
            #tck = UnivariateSpline(x=s, y = interp_quats[:,i], s = None)
            #qi = tck(time*(s[-1]/time[-1]))
            tcks.append(tck)
            
            qs.append(qi)
        
        qs = np.stack(qs).transpose()
        print(qs.shape)
        for i in range(0,qs.shape[0]):
            #print(i)
            qs[i,:] = qs[i,:] / np.linalg.norm(qs[i,:])
        
        Q = uniqueQuaternion(qs)
        print("q shape", Q.shape)
        
        #### END NEW
        
        """
        # Quaternion trajectory
        # Staro, od Leona, dela ampak je ni zvezna trajektorija
        Q = interpQuaternionPath(s, path[:,3:], time, short=True)
        Q = uniqueQuaternion(Q)
        
        # Quaternions with interp spline - this should be done differently
        tcks = []
        qs = []
            
        k = min(5, path.shape[0]-1)
        #for i in range(3,7):
        for i in range(0,4):
            #tck = make_interp_spline(x = s, y = path[:,i], k=3, bc_type=([(1, 0.0)], [(1, 0.0)]))
            tck = make_interp_spline(x = time, y = Q[:,i], k=4, bc_type=([(1, 0.0),(2, 0.0)], [(1, 0.0)]))
            tcks.append(tck)
            qi = splev(time,tck)
            qs.append(qi)
        
        # Quaternion with slerp
        
        qs = np.stack(qs).transpose()
        for i in range(0,qs.shape[0]):
            #print(i)
            qs[i] = qs[i] / np.linalg.norm(qs[i])
        
        Q = uniqueQuaternion(qs)
        """
        
        xi = np.vstack(xis)
        xi = np.transpose(xi)
        
        xi = np.hstack((xi, Q))

        # End new
        
        vi = gradientCartesianPath(xi, time)
        
        self.xi = xi
        self.vi = vi
        
        ai = gradientPath(vi, time)
        
        if only_return_path:
            return xi, vi, ai
                
        # Checking for max speeds
        fac = np.max(np.max(np.abs(vi), axis=0)/self.v_max)
        if fac>1:
            rospy.loginfo("CPath limiting speed! Factor: {}".format(fac))

            s = np.linspace(0,t*fac,N)
            time=np.arange(0,t*fac+self.tsamp,self.tsamp)
            xi = interpCartesianPath(s, path, time)
            vi = gradientCartesianPath(xi, time)        
        
        N = time.size
        
        self.Start()
        tmperr = 0
        kwargs.setdefault('kinematics','Calculated')
        #kwargs.setdefault('state','Commanded')
        kwargs.setdefault('state','Actual')

        xe = np.amax(np.abs(self.TaskDistance(path[0]))/self.v_max)
        if xe>0.02:
            self.Message('Move to path -> CMove',2)
            tmperr = self.CMove(path[0], max(xe*10,0.2), traj='Poly', wait=0, task_space=task_space, FT=FT, **kwargs)
        if tmperr==0:
            
            for xt, vt in zip(xi[:-2],vi[:-2]):
                if self._do_motion_check and self._motion_check_callback is not None:
                    tmperr = self._motion_check_callback(self)
                    if tmperr>0:
                        self._command_int.qdot = np.zeros(self.nj)
                        self._command_int.v = np.zeros(6)
                        self.WarningMessage('Motion check stopped motion')
                        return tmperr
                tmperr = self.GoTo_T(x2t(xt), vt, FT, wait=wait, task_space=task_space, **kwargs)
                if (tmperr>0) and (self._control_strategy not in ['JointPositionTrajectory']):
                    self.WarningMessage('Motion aborted because tmperr>0')
                    return tmperr
                
            tmperr=self.GoTo_T(x2t(xi[-1]), np.zeros(6), FT, wait=wait, task_space=task_space, last_move = True, **kwargs)
            self.Stop()
            self.Message('CPath finished', 2)
        return tmperr

    def TMove(self, x, t, traj='poly', wait=None, FT=None, **kwargs):
        self.Message('TMove -> CMove', 2)
        tmperr = self.CMove(x, t, traj=traj, wait=wait, task_space='Tool', FT=FT, **kwargs)
        return tmperr

    def TLine(self, x, t, wait=None, FT=None, **kwargs):
        self.Message('TLine -> CLine', 2)
        tmperr = self.CLine(x, t, wait=wait, task_space='Tool', FT=FT, **kwargs)
        return tmperr

    def OMove(self, x, t, traj='poly', wait=None, FT=None, **kwargs):
        self.Message('OMove -> CMove', 2)
        tmperr = self.CMove(x, t, traj=traj, wait=wait, task_space='Object', FT=FT, **kwargs)
        return tmperr

    def OMoveFor(self, x, dx, t, traj='poly', wait=None, FT=None, **kwargs):
        self.Message('OMoveFor -> CMoveFor', 2)
        tmperr = self.CMoveFor(x, dx, t, traj=traj, wait=wait, task_space='Object', FT=FT, **kwargs)
        return tmperr

    def OApproach(self, x, dx, t, traj='poly', wait=None, FT=None, **kwargs):
        self.Message('OApproach -> CApproach', 2)
        tmperr = self.CApproach(x, dx, t, traj=traj, wait=wait, task_space='Object', FT=FT, **kwargs)
        return tmperr

    def OLine(self, x, t, wait=None, FT=None, **kwargs):
        self.Message('OLine -> CLine', 2)
        tmperr = self.CLine(x, t, wait=wait, task_space='Object', FT=FT, **kwargs)
        return tmperr
    def OLineFor(self, x, dx, t, wait=None, FT=None, **kwargs):
        self.Message('OLineFor -> CLineFor', 2)
        tmperr = self.CLineFor(x, dx, t, wait=wait, task_space='Object', FT=FT, **kwargs)
        return tmperr

    def JointDistance(self, q, state='Actual'):
        """Distance between current position and q

        Parameters
        ----------
        q : array of floats
            joint position (nj,)
        state : str
            joint positions state (`Actual`, `Command`)

        Returns
        -------
        array of floats
            distance to current q (nj,)
        """
        assert state in ['Actual', 'Commanded']
        q = self.jointvar(q)
        return q-self.GetJointPos(state=state)

    def TaskDistance(self, x, out='x', task_space='World', state='Actual', kinematics='Calculated'):
        """Distance between current pose and x

        Parameters
        ----------
        x : array of floats
            current pose
        out : str
            output form (`x`, `p`, `Q`)

        Returns
        -------
        array of floats
            distance to current pose 
        """
        x = self.spatial(x)

        if x.shape==(4, 4):
            rx = t2x(x)
        elif isvector(x, dim=7):
            rx = x
        elif x.shape==(3, 3):
            rx = map_pose(R=x)
            out = 'Q'
        elif isvector(x, dim=3):
            rx = map_pose(p=x)
            out = 'p'
        elif isvector(x, dim=4):
            rx = map_pose(Q=x)
            out = 'Q'
        else:
            raise ValueError(f'Parameter shape {x.shape} not supported')

        dx = xerr(rx,self.GetPose(task_space=task_space, state=state, kinematics=kinematics))
        if out=='x':
            return dx
        elif out=='Q':
            return dx[3:]
        elif out=='p':
            return dx[:3]
        else:
            raise ValueError(f'Output form {out} not supported')

    def CheckJointLimits(self, q):
        """Check if q in joint range

        Parameters
        ----------
        q : array of floats
            joint position (nj,)

        Returns
        -------
        bool
            True if one joint out of limits
        """
        return np.any(self.q_max-q<0) or np.any(q-self.q_min<0)

    def DistToJointLimits(self, *q):
        """Distance to joint limits

        Parameters
        ----------
        q : array of floats, optional
            joint position (nj,)

        Returns
        -------
        array of floats
            minimal distance to joint limits (nj,)
        array of floats
            distance to lower joint limits (nj,)
        array of floats
            distance to upper joint limits (nj,)
        """
        if len(q)==0:
            q = self._actual_int.q
        else:
            q = self.jointvar(q[0])
        dqUp = self.q_max-q
        dqLow = q-self.q_min
        dq = np.fmin(dqLow,dqUp)
        return dq, dqLow, dqUp

    def BaseToWorld(self, x, *R):
        """Map from robot base frame to world frame

        Supported arguments: pose (7,), Homogenous matrix (4, 4), rotation matrix (3, 3),
        position (3,), twist (6,) and Jacobian matrix (6,nj)

        Parameters
        ----------
        x : array of floats
            argument to map

        Returns
        -------
        array of floats
            mapped argument

        Raises
        ------
        ValueError
            Parameter shape not supported
        """
        p0 = self.TBase[:3,3]
        R0 = self.TBase[:3,:3]
        x = np.asarray(x)
        if len(R)==0:
            if x.shape==(4, 4):
                return self.TBase@x
            elif x.shape==(3, 3):
                return R0@x
            elif isvector(x, dim=7):
                p, R = map_pose(x=x, out='pR')
                return map_pose(p=R0@p+p0, R=R0@R, out='x')
            elif isvector(x, dim=3):
                return R0@x+p0
            elif isvector(x, dim=6):
                return np.hstack((R0@x[:3],R0@x[3:]))
            elif x.shape==(6,self.nj):
                return np.vstack((R0@x[:3,:],R0@x[3:,:]))
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            R = matrix(R, shape=(3, 3))
            x = vector(x, dim=3)
            return map_pose(p=R0@x+p0, R=R0@R, out='pR')

    def WorldToBase(self, x, *R):
        """Map from world frame to robot base frame

        Supported arguments: pose (7,), Homogenous matrix (4, 4), rotation matrix (3, 3),
        position (3,), twist (6,) and Jacobian matrix (6,nj)

        Parameters
        ----------
        x : array of floats
            argument to map

        Returns
        -------
        array of floats
            mapped argument

        Raises
        ------
        ValueError
            Parameter shape not supported
        """
        p0 = self.TBase[:3,3]
        R0 = self.TBase[:3,:3]
        x = np.asarray(x)
        if len(R)==0:
            if x.shape==(4, 4):
                return np.inv(self.TBase)@x
            elif x.shape==(3, 3):
                return R0.T@x
            elif isvector(x, dim=7):
                p, R = map_pose(x=x, out='pR')
                return map_pose(p=(p-p0).T@R0, R=R0.T@R, out='x')
            elif isvector(x, dim=3):
                return (x-p0).T@R0
            elif isvector(x, dim=6):
                return np.hstack((R0.T@x[:3],R0.T@x[3:]))
            elif x.shape==(6,self.nj):
                return np.vstack((R0.T@x[:3,:],R0.T@x[3:,:]))
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            R = matrix(R, shape=(3, 3))
            x = vector(x, dim=3)
            return map_pose(p=R0@x+p0, R=R0@R, out='pR')

    def ObjectToWorld(self, x, *R):
        """Map from object frame to world frame

        Supported arguments: pose (7,), Homogenous matrix (4, 4), rotation matrix (3, 3),
        position (3,), twist (6,) and Jacobian matrix (6,nj)

        Parameters
        ----------
        x : array of floats
            argument to map

        Returns
        -------
        array of floats
            mapped argument

        Raises
        ------
        ValueError
            Parameter shape not supported
        """
        p0 = self.TObject[:3,3]
        R0 = self.TObject[:3,:3]
        x = np.asarray(x)
        if len(R)==0:
            if x.shape==(4, 4):
                return self.TObject@x
            elif x.shape==(3, 3):
                return R0@x
            elif isvector(x, dim=7):
                p, R = map_pose(x=x, out='pR')
                return map_pose(p=R0@p+p0, R=R0@R, out='x')
            elif isvector(x, dim=3):
                return R0@x+p0
            elif isvector(x, dim=6):
                return np.hstack((R0@x[:3],R0@x[3:]))
            elif x.shape==(6,self.nj):
                return np.vstack((R0@x[:3,:],R0@x[3:,:]))
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            R = matrix(R, shape=(3, 3))
            x = vector(x, dim=3)
            return map_pose(p=R0@x+p0, R=R0@R, out='pR')

    def WorldToObject(self, x, *R):
        """Map from world frame to object frame

        Supported arguments: pose (7,), Homogenous matrix (4, 4), rotation matrix (3, 3),
        position (3,), twist (6,) and Jacobian matrix (6,nj)

        Parameters
        ----------
        x : array of floats
            argument to map

        Returns
        -------
        array of floats
            mapped argument

        Raises
        ------
        ValueError
            Parameter shape not supported
        """
        p0 = self.TObject[:3,3]
        R0 = self.TObject[:3,:3]
        x = np.asarray(x)
        if len(R)==0:
            if x.shape==(4, 4):
                return np.inv(self.TObject)@x
            elif x.shape==(3, 3):
                return R0.T@x
            elif isvector(x, dim=7):
                p, R = map_pose(x=x, out='pR')
                return map_pose(p=(p-p0).T@R0, R=R0.T@R, out='x')
            elif isvector(x, dim=3):
                return (x-p0).T@R0
            elif isvector(x, dim=6):
                return np.hstack((R0.T@x[:3],R0.T@x[3:]))
            elif x.shape==(6,self.nj):
                return np.vstack((R0.T@x[:3,:],R0.T@x[3:,:]))
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
        else:
            R = matrix(R, shape=(3, 3))
            x = vector(x, dim=3)
            return map_pose(p=R0@x+p0, R=R0@R, out='pR')

    def manipulability(self, q, **kwargs):
        kwargs.setdefault('task_space','World')
        kwargs.setdefault('task_DOF',np.ones(6))
        J = self.jacobi(q)
        if check_option(kwargs['task_space'], 'World'):
            J = self.WorldToBase(J)
        elif check_option(kwargs['task_space'], 'Robot'):
            pass
        else:
            raise ValueError(f'Task space {kwargs["task_space"]} not supported')
        task_DOF = vector(kwargs['task_DOF'], dim=6)
        Sind = np.where(task_DOF>0)[0]
        JJ = J[Sind,:]
        return np.sqrt(np.linalg.det(JJ@JJ.T))

    def SetGripper(self, grip):
        if isgripper(grip):
            self._gripper = grip
            grip.AttachTo(self)
        else:
            raise TypeError('Object is not gripper')
    
    def SetTCP(self, *tcp, frame='Gripper', hold_pose = 'on'):
        assert hold_pose in ['off', 'on']
        
        if len(tcp)>0:
            x = self.spatial(tcp[0])
            if x.shape==(4, 4):
                TCP = x
            elif x.shape==(3, 3):
                TCP = map_pose(R=x, out='T')
            elif isvector(x, dim=7):
                TCP = map_pose(x=x, out='T')
            elif isvector(x, dim=3):
                TCP = map_pose(p=x, out='T')
            elif isvector(x, dim=4):
                TCP = map_pose(Q=x, out='T')
            else:
                raise ValueError(f'TCP shape {x.shape} not supported')
        else:
            TCP = np.eye(4)
        if check_option(frame,'Robot'):
            newTCP = TCP
        elif check_option(frame,'Gripper'):
            newTCP = self.TCPGripper@TCP
        else:
            raise ValueError(f'Frame {frame} not supported')
        self.TCP = newTCP
        
        if hold_pose == 'on':
            rx, rJ = self.kinmodel(q = self._command_int.q)
            self._command_int.x = self.BaseToWorld(rx)
            self._command_int.v = self.BaseToWorld(rJ@self._command_int.qdot)

    def GetTCP(self, out='T'):
        return map_pose(T=self.TCP, out=out)

    def SetObject(self, *x):
        if len(x)==0:
            xx = self._actual_int.x
        else:
            xx = self.spatial(x[0])
        if xx.shape==(4, 4):
            T = xx
        elif isvector(xx, dim=7):
            T = x2t(xx)
        else:
            raise ValueError(f'Pose shape {xx.shape} not supported')
        self.TObject = T

    def GetObject(self, out='T'):
        return map_pose(T=self.TObject, out=out)

    @abstractmethod
    def SetStrategy(self, strategy):
        pass

    def GetStrategy(self):
        return self._control_strategy

    def isStrategy(self, strategy):
        return check_option(self._control_strategy,strategy)

    def Start(self):
        self._last_control_time = self.simtime()

    def Stop(self):
        self._command_int.mode = 0
        self._command_int.qdot = np.zeros(self.nj)
        self._command_int.mode = np.zeros(6)

    def SetMotionCheckCallback(self, fun):
        self._motion_check_callback = fun

    def EnableMotionCheck(self, check=True):
        self._do_motion_check = check

    def DisableMotionCheck(self):
        self._do_motion_check = False

    def SetCaptureCallback(self, fun):
        self._capture_callback = fun

    def StartCapture(self):
        if not self._do_update:
            self.WarningMessage('Update is not enabled')
        self._t0 = self._tt
        self._do_capture = True
        self.Update()

    def StopCapture(self):
        self._do_capture = False

    def WaitUntilStopped(self, eps=0.001):
        self.GetState()
        while np.linalg.norm(self._actual_int.qdot)>eps:
            self.GetState()

    def Wait(self, t, dt=None):
        if dt is None:
            dt = self.tsamp
        tx = self.simtime()
        imode = self._command_int.mode
        self._command_int.mode = -1
        while self.simtime()-tx<t:
            self.GetState()
            self.Update()
            sleep(dt)
        self._command_int.mode = imode

    def Restart(self):
        self.Stop()
        self.Start()

    def SetUserData(self, data):
        self._command_int.data = data
        self.Update()

    def GetUserdata(self):
        return self._command_int.data

    def Message(self, msg, verb=0):
        if self._verbose>0 and self._verbose>=verb:
            print(f'{self.Name}:{msg}')

    def WarningMessage(self, msg):
         print(f'Warning: {self.Name}:{msg}')

class robot_no_compliance(robot):
    """Dummy robot methods for stiff robot
    """
    def GetJointStiffness(self):
        self.Message('Compliance not suppported',3)
        return np.ones(self.nj)*100000

    def SetJointStiffness(self, stiffness, **kwargs):
        self.Message('Compliance not suppported',3)

    def GetJointDamping(self):
        self.Message('Compliance not suppported',3)
        return np.ones(self.nj)

    def SetJointDamping(self, damping, **kwargs):
        self.Message('Compliance not suppported',3)

    def SetJointSoft(self, softness, **kwargs):
        self.Message('Compliance not suppported',3)

    def GetCartesianStiffness(self):
        self.Message('Compliance not suppported',3)
        return np.ones(6)*100000

    def SetCartesianStiffness(self, stiffness, **kwargs):
        self.Message('Compliance not suppported',3)

    def GetCartesianDamping(self):
        self.Message('Compliance not suppported',3)
        return np.ones(6)

    def SetCartesianDamping(self, damping, **kwargs):
        self.Message('Compliance not suppported',3)

    def SetCartesianSoft(self, softness, **kwargs):
        self.Message('Compliance not suppported',3)

def isrobot(obj):
    return isinstance(obj, robot)
