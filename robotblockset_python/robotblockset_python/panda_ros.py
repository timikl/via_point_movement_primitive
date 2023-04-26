import math
from time import time as tm
from time import sleep

import rospy
import numpy as np
#import logging
import copy # For deep copy of _commanded - check if maybe this import can be removed 

import actionlib

from franka_msgs.msg import *
from franka_msgs.srv import *
from franka_msgs.msg import FrankaState as fs
from robot_module_msgs.msg import ImpedanceParameters, CartesianCommand, JointCommand, JointTrapVelAction,JointTrapVelGoal
from std_msgs.msg import Empty, Float32MultiArray, Bool
from controller_manager_msgs.srv import * # .srv import *
from sensor_msgs.msg import JointState

from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest

from .transformations import *
from .robots import robot
from .robot_models.robot_models import kinmodel_panda
from .validateargs import *
from .ROS.joint_trajectory_interface import JointTrajectory

class joint_compliance():
    def __init__(self):
        self.K = None
        self.D = None

class cartesian_compliance():
    def __init__(self):
        self.Kp = None
        self.Kr = None # added later, is cool ?
        self.R = None
        self.D = None

class collision_thresholds():
    def __init__(self):
        self.F = None
        self.T = None
        self.tq = None

class panda_ros(robot):
    def __init__(self, ns, init_node= True, multi_node = False, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller'):
        """ parameters:
        init_node : if runs rospy.init_node. does not need to be run if using flexbe"""
        
        #self.Logger = logging.getLogger(__name__) 
        robot.__init__(self)
        
        self.init_complete = 0 # Before it works, we must set the initial _command.q and _actual.q . We will track this variable in self.state_callback, so we can set these params at startup
        
        self.Name = ns
        self.Base_link_name = self.Name + '/' + self.Name + '_link0' 
        
        # Load all the controllers
        self._control_strategy = None # 'JointPosition'
        self.switch_controller_topic = "/%s/controller_manager/switch_controller"%self.Name
        self.switch_controller_proxy = rospy.ServiceProxy(self.switch_controller_topic, SwitchController)
        self.load_controller_topic = "/%s/controller_manager/load_controller"%self.Name
        self.load_controller_proxy = rospy.ServiceProxy(self.load_controller_topic, LoadController)
        
        #controllers_to_load = ['joint_impedance_controller', 'cartesian_impedance_controller', 'position_joint_trajectory_controller']
        controllers_to_load = ['cartesian_impedance_controller', 'position_joint_trajectory_controller']
    
        assert start_controller in controllers_to_load
        
        self.controller_names = {'JointPositionTrajectory': 'position_joint_trajectory_controller',
                                 'JointImpedance': 'joint_impedance_controller',
                                 'CartesianImpedance':'cartesian_impedance_controller'}
        
        self.controller_names_inv = {v: k for k, v in self.controller_names.items()} # The inversion
        
        for controller in controllers_to_load:
            self._load_controller(desired_controller = controller)
    
        # Stop all controllers
        self._switch_controller(start_controller = [], stop_controller=controllers_to_load, strictness = 1)
        
        # Start the desired controller
        stop_controllers = [controller for controller in controllers_to_load if controller != start_controller]
        self._switch_controller(start_controller = [start_controller], stop_controller=stop_controllers, strictness = 1)

        # Robot specs init (copied from matlab panda_spec.m)
        self.nj=7 # Num of joints
       
        self.q_home= np.array([0, -0.2, 0, -1.5, 0, 1.5, 0.7854]) # home joint configuration
        self.q_max=np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]) # upper joint limits
        self.q_min=np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]) # lower joint limits
        self.qdot_max=np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]) # maximal joint velocities
        self.qddot_max=np.array([15, 7.5, 10, 12.5, 15, 20, 20]) # maximal joint accelerations

        #self.v_max= np.array([1.5, 1.5, 1.5, 2, 2, 2]) #  maximal task velocities
        self.v_max= np.array([1.3, 1.3, 1.3, 1.8, 1.8, 1.8]) #  maximal task velocities
        self.a_max= np.array([20, 20, 20, 20, 20, 20]) #  maximal task accelerations

        # TO be set better !
        #self._command = _command()
        self._command_int.mode = 2 
        
        self.last_control_time = tm() # tm()
        
        #self.tsamp = 0.01 
        #self.tsamp = 0.005
        self.tsamp = 1./300.
                
        self.joint_compliance = joint_compliance()
        self.cartesian_compliance = cartesian_compliance()
        self.collision_thresholds = collision_thresholds()
                
        self.joint_compliance.K = np.array([1200, 1200, 1200, 600, 250, 250, 50])
        self.joint_compliance.D = np.array([25, 25, 25, 25, 10, 10, 10])
        
        self.empty_impedance_parameters = ImpedanceParameters()
        self.empty_JointCommand = JointCommand()
        
        # Standard default values
        self.cartesian_compliance.Kp =np.ones(3,dtype=float)*2000
        self.cartesian_compliance.Kr = np.ones(3, dtype=float)* 50
        self.cartesian_compliance.R = np.eye(3, dtype= float)
        self.cartesian_compliance.D = 2
        self.empty_CartesianCommand =  CartesianCommand()
        
        self.collision_thresholds.F = np.array([10, 10, 10])
        self.collision_thresholds.T = np.array([10, 10, 10])
        self.collision_thresholds.tq = np.array([10, 10, 10, 10, 10, 10, 10])
        # These values are set only in here, for the time being. should be sent to controller ! 
        
        if init_node:
            try:
                rospy.init_node("FrankaHandler_{}".format(self.Name), anonymous = True if multi_node else False ) 
            except rospy.ROSException as e:
                rospy.loginfo("Skipping node init because of exception: {}".format(e))
        self.logger_svc_topic = "/%s/franka_control/set_logger_level"%self.Name
        self.logger_svc_publisher = rospy.ServiceProxy(self.logger_svc_topic, SetLoggerLevel)
        
        self.joint_command_publisher = rospy.Publisher('/%s/joint_impedance_controller/command'%self.Name, JointCommand, queue_size=1)
        
        self.joint_trajectory_interface = JointTrajectory(controller_ns = self.Name)
        
        self.cart_activate_topic = "/%s/cartesian_impedance_controller/activate"%self.Name
        self.cart_activate_publisher =  rospy.Publisher(self.cart_activate_topic, Bool, queue_size=1)
        
        self.cart_command_topic = "/%s/cartesian_impedance_controller/command"%self.Name
        self.cartesian_command_publisher = rospy.Publisher(self.cart_command_topic, CartesianCommand, queue_size=1)
        
        self.cart_null_q_topic = "/%s/cartesian_impedance_controller/nullspace_q"%self.Name
        self.cart_null_k_topic = "/%s/cartesian_impedance_controller/nullspace_stiff"%self.Name
        
        self.cart_null_q_publisher = rospy.Publisher(self.cart_null_q_topic, Float32MultiArray, queue_size=1, latch = False)
        self.cart_null_k_publisher = rospy.Publisher(self.cart_null_k_topic, Float32MultiArray, queue_size=1, latch = False)
        
        self.cart_reset_topic = "/%s/cartesian_impedance_controller/reset_target"%self.Name
        self.reset_target_svc = rospy.Publisher(self.cart_reset_topic, Empty, queue_size=1)
        
        self.cart_stiff_topic = "/%s/cartesian_impedance_controller/stiffness"%self.Name
        #self.cart_stiff_proxy = rospy.ServiceProxy({self.cart_command_topic: CartesianCommand, self.cart_reset_topic: Empty, self.cart_stiff_topic: ImpedanceParameters} )
        self.cart_stiff_publisher = rospy.Publisher(self.cart_stiff_topic, ImpedanceParameters, queue_size = 1)
        
        self.unload_controller_topic = "/%s/controller_manager/unload_controller"%self.Name
        self.unload_controller_proxy = rospy.ServiceProxy(self.unload_controller_topic, UnloadController)
        
        self.joint_impedance_franka_topic =  "/" + self.Name + "/franka_control/set_joint_impedance"
        self.joint_impedance_franka_proxy = rospy.ServiceProxy(self.joint_impedance_franka_topic, SetJointImpedance)        
        
        self.cart_impedance_franka_topic = "/" + self.Name + "/franka_control/set_cartesian_impedance"
        self.cart_impedance_franka_proxy = rospy.ServiceProxy(self.cart_impedance_franka_topic, SetCartesianImpedance)
        
        self.K_frame_topic = "/%s/franka_control/set_K_frame"%self.Name
        #rospy.loginfo("Waiting for K frame service")
        #rospy.wait_for_service(self.K_frame_topic)
        self.K_frame_proxy = rospy.ServiceProxy(self.K_frame_topic, SetKFrame)
        
        # Collision behavior
        self.force_torque_limits_topic = "/%s/franka_control/set_force_torque_collision_behavior"%self.Name
        self.force_torque_limits_proxy = rospy.ServiceProxy(self.force_torque_limits_topic, SetForceTorqueCollisionBehavior)
        
        # Full Collision behavior (also force/tq/jointtq limits during acceleration)
        self.full_force_torque_limits_topic = "/%s/franka_control/set_full_collision_behavior"%self.Name
        self.full_force_torque_limits_proxy = rospy.ServiceProxy(self.full_force_torque_limits_topic, SetFullCollisionBehavior)
        
        self.EE_frame_topic = "/%s/franka_control/set_EE_frame"%self.Name
        #rospy.loginfo("Waiting for EE frame service")
        #rospy.wait_for_service(self.EE_frame_topic)
        self.EE_frame_proxy = rospy.ServiceProxy(self.EE_frame_topic, SetEEFrame)       
  
        self.error_recovery_action_client = actionlib.SimpleActionClient('/%s/franka_control/error_recovery'%self.Name, ErrorRecoveryAction)
        #self.error_recovery_action_client.wait_for_server()
        #self.joint_action_topic = "/" + str(self.Name) + "/joint_impedance_controller/move_joint_trap"
        self.joint_action_topic =  "/" + str(self.Name) + "/joint_impedance_controller/move_joint_trap"
        self.joint_action_client = actionlib.SimpleActionClient(self.joint_action_topic, JointTrapVelAction)
        self.joint_impedance_reset_client = rospy.Publisher("/" + str(self.Name) + "/joint_impedance_controller/reset_target", Empty, queue_size = 1)
        
        self.state = None # If the controller is dead, there will be no state callback
        self.last_states_callback = tm() # Last time we got FrankaState (inside GetStateCallback() )
        self._last_update = tm() # Last time we did GetState()
        self.franka_state_topic = "/%s/franka_state_controller/franka_states"%self.Name
        self.franka_state_subscriber = rospy.Subscriber(self.franka_state_topic, FrankaState, self.GetStateCallback);sleep(0.2)
        #print("/%s/franka_state_controller/franka_states"%self.Name, FrankaState, self.GetStateCallback)
        
        # Anonymous=True, because we may want to run 2 robots at once (panda_1, panda_2)
        
        #rospy.spin()
        
        # Initialize _actual state values and, because the robot just started,
        # initialize _command (commanded state) value to be the same as actual values.
        # Wait until GetState returns a valid panda state object
        while self.state is None:
            sleep(0.01)
        #if init_frankadesk_gripper_TCP:
        #    self.SetFrankadeskTCP()
        #else:
        self.Init_panda()
        
        self.TCPGripper = np.eye(4) # self.TCPGripper was not defined before. DEFINE IT !
        
        #TODO: add auto init that cheks which controller is running
        if 0: 
            # Set the TCP AFTER starting the controller !
            if (self.TCPGripper == np.eye(4)).all():
                # Get TCP from franka_state
                #tcp_setting = np.array(self.state.F_T_EE) # Read TCP that is set in panda-desk
                #tcp_setting = np.reshape(tcp_setting, newshape = (4,4), order = 'F')
                #self.TCPGripper = tcp_setting

                # "Screwdriver" tool by default
                #print("Panda_ros TCP not defined, setting as screwdriver tool by default")
                self.TCPGripper=np.array([[0.7071, 0.7071, 0, 0],
                                [-0.7071, 0.7071, 0, 0],
                                [0, 0, 1, 0.1034],
                                [0, 0, 0, 1]])
                try:
                    self.ReadTCPFromFrankaROS()
                except:
                    self.error_recovery(force = False, reset_target = False)
                    self.ReadTCPFromFrankaROS() # This calls setTCP internally
            
        # Recorded stuff
        self.recorded_joint_trajectories = [] # Record the GOAL sent to the jointpositionTrajectory controller
        self.recorded_jmove_trajectories = [] # In Jmove, record the calculated qs
        self.recorded_cmove_trajectories = [] # In CMove, record the calculated xs
        self.reset_recorded_states() # Makes an empty list self.recorded_states_list, where upon callback literally the entire franka state is saved
        
    def SetCartImpContNullspace(self, q, k):
        if type(q) in [list, tuple]:q=np.array(q)
        if type(k) in [list, tuple]:k=np.array(k)
        
        assert q.shape[0] == 7
        assert k.shape[0] == 7
        
        if self._control_strategy == 'CartesianImpedance':
            q_msg = Float32MultiArray()
            k_msg = Float32MultiArray()
            
            q_msg.data = q
            k_msg.data = k
            
            self.cart_null_q_publisher.publish(q_msg)
            self.cart_null_k_publisher.publish(k_msg)

            self.cart_null_q = q
            self.cart_null_k = k
            rospy.loginfo("Cart imp nullspace is set. {}\n {}".format(q, k))
            
        else:
            rospy.logwarn("SetCartesianNullspace : Strategy not supported. {}".format(self.Name))
            
    def GetCartImpContNullspace(self):
        return self.cart_null_q, self.cart_null_k
    
    def Init_panda(self):
        """
        Init Initializes the robot
        %
        %   - call robot.Update
        %   - set commanded values to actual values
        %   - set the initial time robot.t0"""
        
        self.InitObject_panda()
        self.ResetCurrentTarget()
        self.t0 = self.tt
        #self.ResetTime()
            
    def InitObject_panda(self):
        # Hack. this is in robots.py but not initialized fast enough, before SetTCP
        self.Expected_TCPFrame = ['Gripper','Robot']
        self.Default_TCPFrame = 'Gripper'
        
        self._command_int.q = np.zeros(self.nj)
        self._command_int.qdot=np.zeros(self.nj)
        self._command_int.trq=np.zeros(self.nj)
        self._command_int.p=np.zeros(3)
        self._command_int.R=np.eye(3)
        self._command_int.v=np.zeros(6)           
        self._command_int.FT=np.zeros(6)            
        #self._command_int.data=None;   
        
        self._actual_int.q=np.zeros(self.nj)
        self._actual_int.qdot=np.zeros(self.nj)
        self._actual_int.trq=np.zeros(self.nj)
        self._actual_int.p=np.zeros(3)
        self._actual_int.R=np.eye(3)
        self._actual_int.v=np.zeros(6)
        self._actual_int.FT=np.zeros(6)
        self._actual_int.trqExt=np.zeros(self.nj)
        self.Default_AddedTrq = np.zeros(self.nj)
        self.Default_AddedFT = np.zeros(6)
        
        self.ACTIVE = False
        
        # Read TCP from franka ros
        self.ReadTCPFromFrankaROS()
    
    def ResetCurrentTarget(self, send_msg = True):
        if send_msg == True:
            c = self._control_strategy
            if c in ['CartesianImpedance', 'CartesianVelocity', 'CartesianPose', 'JointImpedance']:
                msg = Empty()
                self.reset_target_svc.publish(msg)
                #print("Panda_ROS publishing reset target msg")
            elif c in ['JointImpedance']:
                msg = Empty()
                self.joint_impedance_reset_client.publish(msg)
                0
            elif c in ['JointPosition', 'JointVelocity', 'JointPositionTrajectory']:
                print("Target reset msg not sent - %s strategy does not support it"%c)
            else:
                print("Target reset msg not sent! Strategy %s not supported"%c)
        sleep(0.1)
        self.GetState()
        sleep(0.1)
        
        #self._command_int.q = copy.deepcopy(self._actual_int.q)
        #self._command_int.qdot = copy.deepcopy(self._actual_int.qdot)
        #self._command_int.x = copy.deepcopy(self._actual_int.x)
        #self._command_int.v = copy.deepcopy(self._actual_int.v)
        
        #self._command_int.q = copy.deepcopy(self._actual_int.q)
        # Hack to stop shaking
        a = np.array(self._actual_int.q)
        b = np.array(self._command_int.q)
        q_error = np.abs(a-b)
        q_error_sum = np.sum(q_error)
        reset = 0
        if q_error_sum > 0.02:
            reset = 1
            self._command_int.q = copy.deepcopy(self._actual_int.q)
        
        #print("RST:actual: {}".format(self._actual_int.q))
        #print("RST:command: {}".format(self._command_int.q))
        #print("RST:qerr: ", q_error)
        #print("RST: reset:", reset)
        
        sleep(0.1)
        
        self._command_int.qdot = self._actual_int.qdot*0
        self._command_int.trq = self._actual_int.qdot*0
        self._command_int.p = self._actual_int.p
        self._command_int.R = self._actual_int.R
        self._command_int.v = np.zeros((6,1))
        self._command_int.FT = np.zeros((6,1))
        
        # Added, not in matlab
        self._command_int.x = copy.deepcopy(self._actual_int.x)
        
        self.Update()
        sleep(0.01)
    
    def ResetTime(self):
        """ Resets the time """
        self.GetState()
        self.t0 = self.tt
        self.Update()
        
    def GetState(self):
        """%Update  Reads all robot variables and stores them
            %
            % Required variables to be updated:
            %   - robot.actual_int.q    actual joint positions (nj x 1)
            %   - robot.actual_int.qdot actual joint velocities (nj x 1)
            %   - robot.actual_int.p    actual task position (3 x 1)
            %   - robot.actual_int.R    actual task orientation (3 x 3)
            %   - robot.actual_int.v    actial task velocity (6 x 1)
            %   - robot.actual_int.FT   end-effector forces/torques (6 x 1)
            %   - robot.tt              time
            %
            % Usage:
            %       GetState;
            %
        """
        
        #st = copy.deepcopy(self.state)
        st = self.state
        #t = tm() #tm()
        t = tm()
        if 1:
        #if (t - self._last_update) > self.tsamp:
            
            self.tt = self.state.header.stamp.secs
            #self._actual_int.q = np.array(st.q, dtype = np.float16)
            self._actual_int.q = (round(st.q[0],6), round(st.q[1],6), round(st.q[2],6), round(st.q[3],6), round(st.q[4],6), round(st.q[5],6), round(st.q[6],6))

            self._actual_int.qdot = st.dq
            self._actual_int.trq = st.tau_J
            
            #### I ADDED CARTESIANIMPEDANCE HERE !!! , its not present in original matlab version
            if self._control_strategy in ['CartesianVelocity', 'CartesianImpedance']:
                self._command_int.q = st.q
                self._command_int.qdot = st.dq_d
                
                #self._command_int.q = st.q_d
                #self._command_int.qdot = st.dq_d
                T_D = np.reshape(st.O_T_EE_d, (4,4), order = 'F')
                self._command_int.p = t2p(T_D)
                self._command_int.R = t2r(T_D)
                
            T = np.reshape(st.O_T_EE, (4,4), order = 'F')
            self._actual_int.x = t2x(T)
            self._actual_int.p = T[0:3, -1]
            self._actual_int.R = T[0:3,0:3]
            J = self.jacobi(st.q)
            self._actual_int.v = J*st.dq
            
            self.Tstiff = np.reshape(st.EE_T_K, (4,4))
            #self._actual_int.FT = frame2world(st.K_F_ext_hat_K, self.Tstiff, 1)
            self._actual_int.F = self._actual_int.FT[0:2]
            self._actual_int.T = self._actual_int.FT[2:]
            self._actual_int.trqExt = st.tau_ext_hat_filtered
            
            # Get safety status
            self.joint_contacts = st.joint_contact
            self.joint_collisions = st.joint_collision
            self.cartesian_contacts = st.cartesian_contact
            self.cartesian_collisions = st.cartesian_collision
                        
            #self._actual_int.q = st.q
            #self._actual_int.qdot = st.dq
            #x, J = self.kinmodel(q=st.q, tcp = self.TCP)
            #self._actual_int.x = x
            #self._actual_int.v = None
            #self._actual_int.trq = st.tau_J
            #self._actual_int.FT = st.K_F_ext_hat_K
            #self._actual_int.trqExt = st.tau_ext_hat_filtered
            
            self._last_update = tm() # Do not change !

        return st
    
    def GetCollisions(self):
        
        qcol = self.joint_collisions
        xcol = self.cartesian_collisions
        return qcol, xcol
      
    def ReadTCPFromFrankaROS(self):
        """Reads the currently set TCP in franka desk and sets it as default inside the panda_Ros object"""
        
        frankadesk_TCP = self.state.F_T_EE
        self.TCPGripper=np.reshape(frankadesk_TCP, newshape = (4,4), order = 'f')
        #self.SetTCP(TCP = None, hold_pose = 'off') # Set TCP to none, then it reads self.TCPGripper
        self.SetTCP(TCP = None, TCPFrame = 'Gripper', hold_pose = 'off') # Set TCP to none, then it reads self.TCPGripper
        
        #rospy.loginfo("TCP set to:")
        #rospy.loginfo(self.TCPGripper)
        
    def SetTCP(self,TCP, send_msg_to_franka_control = False, **kwargs):
        # Sets the TCP at the controller, which is temporarily stopped
        k = kwargs.keys()
        
        EEFrame = 'Flange'
        if 'EEFrame' in k:
            EEFrame = kwargs['EEFrame']
            
        TCPFrame = self.Default_TCPFrame
        if 'TCPFrame' in k:
            TCPFrame = kwargs['TCPFrame']
            
        assert TCPFrame in self.Expected_TCPFrame
        assert EEFrame in ['Flange', 'Nominal']
        
        if TCP is None:
            if TCPFrame  == 'Gripper':
                newTCP = self.TCPGripper
            else:
                newTCP = np.eye(4)
        else:
            if TCPFrame == 'Gripper':
                newTCP = self.TCPGripper / self.TCP
            else:
                newTCP = self.TCP
            
            assert TCP.shape == (4,4)
            
            if TCP.shape == (4,4):
                newTCP = TCP
            else:
                print("SetTCP: wrong target pose form")
                
            if TCPFrame =='Robot':pass
            if TCPFrame == 'Gripper':
                newTCP = self.TCPGripper * newTCP
            else:
                print("SetTCP : Not supported TCP base frame")
                
        self.TCP = newTCP
        [rp,rR, rJ] = self.kinmodel(q = self._command_int.q, out='pR') # , tcp = self.TCP
        self._command_int.p = rp
        self._command_int.R = rR
        #self._command_int.v = rJ*self._command_int.qdot  # Should be uncommented 
        if send_msg_to_franka_control:
            # First stop currently running controller
            self.Stop_controller()
            set_ee_request_msg = SetEEFrameRequest()
            if EEFrame == 'Nominal':
                NE_TCP = self.TCPGripper/newTCP
            else:
                NE_TCP = newTCP
                
            tcp_for_msg = np.reshape(NE_TCP, newshape = 16, order = 'A')
            print(tcp_for_msg.shape, tcp_for_msg)
            set_ee_request_msg.NE_T_EE = tcp_for_msg
            response = self.EE_frame_proxy.call(set_ee_request_msg)
            print("Response to SetTCP:", response)
            
            self.Start_controller()
        self.GetState()
        self.Update()
        
    def kinmodel(self, q= None, tcp=None, out='x'): 
        if tcp is None:
            tcp = self.TCP
         
        if q is None:
            self.GetState()
            q = self._actual_int.q
        return kinmodel_panda(q, tcp=tcp, out=out)  
    
    def Activate_controller(self):
        """Send a Bool msg to the activate topic of the cart imp controller """
        assert self._control_strategy == 'CartesianImpedance'
        
        msg = Bool()
        msg.data = True
        self.cart_activate_publisher.publish(msg)
    
    def GetCartesianDamping(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def GetCartesianStiffness(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def GetJointDamping(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def GetJointStiffness(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def SetCartesianDamping(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def SetCartesianSoft(self):
        """ Make the robot SOFT-o ! """
        Kp = [0,0,40]
        Kr = [0,0,0]
        self.SetReconcycleCartesianCompliance(Kp = Kp, Kr = Kr, hold_pose = 'on')
        return 0
    
    def Soft(self): # Alias for SetCartesianSoft()
        self.SetCartesianSoft()
        return 0
    
    def SetCartesianStiff_helper(self, Kp = [2000, 2000, 2000],Kr = [50,50,50],m=1, n=1, D=2, hold_pose = 'on'):        
        if not(0<=m<=1.5):
            rospy.logwarn("Factor 'n' must be between 0 and 1.5! Setting n to 1.")
            m=1       
               
        if not(0<=n<=2):
            rospy.logwarn("Factor 'm' must be between 0 and 2! Setting m to 1.")
            n=1
        
        Kp=np.array(Kp)*m
        Kr=np.array(Kr)*n      
       
        self.SetReconcycleCartesianCompliance(Kp = Kp, Kr = Kr, D=D, hold_pose = hold_pose)
        return 0
    
    def Stiff(self): # Alias for SetCartesianStiff()
        self.SetCartesianStiff_helper()
        return 0
    
    def SetCartesianStiffness(self, Kp = None, Kr= None, R = None, D = None, hold_pose = 'on'):
        out = self.SetReconcycleCartesianCompliance(Kp = Kp, Kr= Kr, R = R, D = D, hold_pose = hold_pose)
        return out
    
    def SetJointDamping(self, D):
        D = np.array(D)
        if D.shape[0] == 7:
            self.joint_compliance.D = D
            if self._control_strategy in ['JointImpedance']:
                self.GetState()
                self.GoTo_q(q = self.q, qdot=np.zeros(7), trq = np.zeros(7), wait= 0.01)
                return 0
            else:
                print("Panda_ros - SetJointDamping called but control strategy is not JointImpedance. Only setting damping internally.")
                return 0
        print("panda_ros - SetJointDamping - invalid D shape")
        return None
    
    def SetJointSoft(self):
        rospy.loginfo("Function not implemented in panda_ros!")
        return None
    
    def SetJointStiffness(self, K):
        K = np.array(K)
        if K.shape[0] == 7:
            self.joint_compliance.K = K
            if self._control_strategy in ['JointImpedance']:
                # Send message to controller (set the current position as commanded)
                self.GetState()
                self.GoTo_q(q = self.q, qdot=np.zeros(7), trq = np.zeros(7), wait= 0.01)
                return 0
            else:
                print("Panda_ros - SetJointStiffness called but control strategy is not JointImpedance. Only setting compliance internally.")
                return 0
        print("panda_ros - SetJointStiffness - invalid K shape")
        return None
    
    def SetJointImpedanceFranka(self, stiffness):
        """ Sets the joint impedance of the"""
        request = SetJointImpedance()
        request.joint_stiffness = stiffness
        try:
            if 1:
            #if self.K_frame_proxy.is_available(self.K_frame_topic):
                success = self.joint_impedance_franka_proxy.call(request)
                if success.success == True:
                    rospy.loginfo("Franka joint imp. successful: {0}".format(success.success))
                    return 0
                elif success.success == False:
                    rospy.loginfo("Franka joint imp. successful: {0}".format(success.success))
                    return 1
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo(e)
            return 1
        return 0
    
    def SetStrategy(self):
        return None
    
    def jacobi(self, q = None):
        p, R, J = self.kinmodel(q = q, out = 'pR')
        return J
    
    def GetStateCallback(self, data):
        self.state = data
        #self.state = copy.deepcopy(data)
        
        cur_t = tm()
        self.tcall = cur_t - self.last_states_callback
        self.last_states_callback = cur_t
        
        if not self.init_complete:
            self._actual_int.q = data.q
            self._command_int.q = copy.deepcopy(self._actual_int.q)
            
            # OLD:
            #self._actual_int.qdot = data.dq
            #self._command_int.qdot = self._actual_int.qdot
            #self._command_int.q = self._actual_int.q
            #self._command_int.qdot = self._actual_int.qdot
            
            self.init_complete = 1
            #self.GetState()
        
        #self.GetState()
    
    def CMove(self, x, t, traj='poly', wait=None, FT=None, **kwargs):
        
        #HACK: to just use the joint trap vel action server for entire trajectory generation, if using jointimpedance controller
        if self._control_strategy in ['JointImpedanceAction']:
            # Calculating the required q to reach a position x using inverse kinematics
            q_target = None
            x=np.array(x)
            if x.shape == (3,):
                # Only the position is specified, we hold the current rotation and add it to the x vector to turn it from len 3 to len 7
                x = np.concatenate((x, self._command_int.x[3:]))
            elif x.shape == (4,4):
                x = t2x(x)
            elif x.shape==(7,):
                pass
            else:
                print("CMove got invalid x shape:", x.shape)
                print("Aborting move")
                return 0
            
            q_target = self.x2q_invkin(x)
            
            if q_target is None:
                rospy.loginfo("CMove failed to get inverse kinematics solution. Make shorter moves or increase iteration limit in robots.py")
            else:
                self.JMove(q = q_target, t=t, **kwargs)
            return 0
        
        #return super(panda_ros, self).CMove(x,t,traj,wait,FT,**kwargs)
        return robot.CMove(self,x,t,traj,wait,FT,**kwargs)

    def x2q_invkin(self, x, q_last = None, return_trajectory = False, dq_factor = 10):
        
        if return_trajectory:
            q_trajectory = []
            
        if q_last is None:
            q_last = self.q
        """ Function to calculate joints from an x"""
        q_target = None
        # We can set the limit to 15000 because it will break out of the loop when error is low enough anyway
        rp = copy.deepcopy(x[:3]) # reference (desired) position
        rR = copy.deepcopy(q2r(x[3:])) # Reference (desired) rotation
        RC = np.kron(np.eye(2),self.TBase[:3,:3]).T
        for i in range(0,10000):
            #t1 = tm()
            p, R, J = self.kinmodel(q = q_last, out='pR')
            Jp = np.linalg.pinv(J)
            ep = rp-p # position error
            #print(i, "position error", ep[0])
            
            eR = qerr(r2q(rR@R.T))
            ee = np.hstack((ep, eR))
            ee = RC@ee
            t2 = tm() #tm()
            #rospy.loginfo("x2q 1st part: {}".format(t2-t1))
            
            u =Jp@ee
            u = np.clip(u, -self.qdot_max, self.qdot_max)
            
            #rq = qq+u*self.tsamp
            #q_last = q_last + 0.1*Jp@ee # New joint configuration that is closer to the desired x
            q_last = q_last + u*self.tsamp*dq_factor
            #t3 = tm()
            #rospy.loginfo("x2q 2nd part: {}".format(t3-t2))

            if return_trajectory: q_trajectory.append(q_last)
            
            if ((np.sum(abs(ep)) < 0.001) and (np.sum(abs(eR))< 0.01)):
                #rospy.loginfo("x2q finished in {} iterations".format(i))
                q_target = q_last
                break
            
        return q_target
    
    def GoTo_X(self, x, xdot, trq, wait, **kwargs):
        """GoTo_X Update task position and orientation for task controller and wait for time t
            Inputs:
            x        task position,quaternion (1 x 7) (in robot CS)
            xdot   task twist (6 x 1) (in robot CS)
            trq      EE wrench (6 x 1) (in robot CS)
            wait    wait time after update
                
            Options:
            Kp  position stiffness
            Kr  orientation stiffness
            R   stiffness rotation
            D   damping factor
         """
        
        #print("PandaRobot l153 - GoTo_x got x_command ", x)
        #if rospy.is_shutdown():
        #    print("panda_RBS - ROSPY is shutdown, breaking out of function (GoTo_x)") 
        #    return

        if self._control_strategy in ['CartesianPose', 'CartesianVelocity']:
        
            # TODO : check if values are numeric (currently we only check if x, xdot are vectors)
            if not isvector(x, dim=7):
                raise Exception("%s : GoTo_x: NAN x value"%self.Name)
            if not isvector(xdot, dim=6):
                raise Exception("%s : GoTo_x: NAN xdot value"%self.Name)
            if not isvector(trq, dim = 6):
                raise Exception("%s : GoTo_x: NAN trq value"%self.Name)
            
            #cmd_msg = rosmessage(robot.command_pub)  # FIX
            
            #cmd_msg = CartesianCommand()
            cmd_msg = self.empty_CartesianCommand
            # TO DO : time_from_start, add acc, ft, impedance
            cmd_msg.pose.position.x = x[0]
            cmd_msg.pose.position.y = x[1]
            cmd_msg.pose.position.z = x[2]
            
            cmd_msg.pose.orientation.w = x[3]
            cmd_msg.pose.orientation.x = x[4]
            cmd_msg.pose.orientation.y = x[5]
            cmd_msg.pose.orientation.z = x[6]
            
            cmd_msg.vel.linear.x = xdot[0]
            cmd_msg.vel.linear.y = xdot[1]
            cmd_msg.vel.linear.z = xdot[2]
            cmd_msg.vel.angular.x = xdot[3]
            cmd_msg.vel.angular.y = xdot[4]
            cmd_msg.vel.angular.z = xdot[5]
            
            # Publish
            self.cartesian_command_publisher.publish(cmd_msg)  
            
            self.GetState()
            self.Update()

            #self._command_int.x = x     # Tega ni v matlabu in tudi tu bom zakomentiral
            #self._command_int.vel = xdot
            #self._command_int.trq = trq
                        
            while (tm() - self.last_control_time) < wait:
                #print(tm() - self.last_control_time)
                #print(wait)
                #print("---")
                0#pass
            
            self.last_control_time= tm() # tm() 
            
        elif self._control_strategy == 'CartesianImpedance':
            
            kwargs.setdefault('R',self.cartesian_compliance.R)
            kwargs.setdefault('D',self.cartesian_compliance.D)
            kwargs.setdefault('Kp',self.cartesian_compliance.Kp)
            kwargs.setdefault('Kr',self.cartesian_compliance.Kr)
        
            R = kwargs['R']
            D = kwargs['D']
            Kp =kwargs['Kp']
            Kr =kwargs['Kr']
            
            # TODO : check if values are numeric (currently we only check if x, xdot are vectors)
            if not isvector(Kp, dim=3):
                raise Exception("%s : GoTo_x: NAN Kp value"%self.Name)
            if not isvector(Kr, dim=3):
                raise Exception("%s : GoTo_x: NAN Kr value"%self.Name)
                
            if not ismatrix(R, shape = (3,3)):
                raise Exception("%s : GoTo_x: wrong shape R value"%self.Name)
            if not (D>=0 and D<=2) :
                raise Exception("%s : GoTo_x: D out of bounds"%self.Name)
                
            if not isvector(x, dim=7):
                raise Exception("%s : GoTo_x: NAN x value"%self.Name)
            if not isvector(xdot, dim=6):
                raise Exception("%s : GoTo_x: NAN xdot value"%self.Name)
            if not isvector(trq, dim = 6):
                raise Exception("%s : GoTo_x: NAN trq value"%self.Name)
                
            cmd_msg = self.empty_CartesianCommand            
            cmd_msg.pose.position.x = x[0]
            cmd_msg.pose.position.y = x[1]
            cmd_msg.pose.position.z = x[2]
            
            cmd_msg.pose.orientation.w = x[3]
            cmd_msg.pose.orientation.x = x[4]
            cmd_msg.pose.orientation.y = x[5]
            cmd_msg.pose.orientation.z = x[6]
            
            # TODO : moznost izklopa, npr. robot.user.SEND_VELOCITY = 1
            cmd_msg.vel.linear.x = xdot[0]
            cmd_msg.vel.linear.y = xdot[1]
            cmd_msg.vel.linear.z = xdot[2]
            cmd_msg.vel.angular.x = xdot[3]
            cmd_msg.vel.angular.y = xdot[4]
            cmd_msg.vel.angular.z = xdot[5]
            
            
            cmd_msg.ft.force.x = trq[0]
            cmd_msg.ft.force.y = trq[1]
            cmd_msg.ft.force.z = trq[2]
            
            cmd_msg.ft.torque.x = trq[3]
            cmd_msg.ft.torque.y = trq[4]
            cmd_msg.ft.torque.z = trq[5]
            
            # Calculate stiffness matrix
            trM = np.diag(Kp)
            rotM = np.diag(Kr)
            # Rotate
            trK = R * trM * np.transpose(R)
            rotK = rotM
            # Damping
            trD = R * 2 * np.sqrt(trM)*np.transpose(R)
            rotD = D*np.sqrt(rotM)
            
            # Check if any is NaN
            if np.isnan(trM).any() or np.isnan(rotM).any() or np.isnan(trK).any() or np.isnan(rotK).any() or np.isnan(trD).any() or np.isnan(rotD).any():
                raise Exception("%s: GoTo_x: trM or rotM or trK or rotK or trD or rotD - NaN error"%self.Name)
                
            stiffness = self.empty_impedance_parameters
            stiffness.n = 9
            stiffness.k = np.concatenate([np.reshape(trK, (9,1)), np.reshape(rotK, (9,1))])
            #rospy.loginfo("{0}".format(stiffness.k))
            stiffness.d = np.concatenate([np.reshape(trD, (9,1)), np.reshape(rotD, (9,1))])
            
            cmd_msg.impedance = stiffness
            
            # Update values to reflect current values of stiffness and positions.
            self.cartesian_compliance.R = R
            self.cartesian_compliance.D = D
            self.cartesian_compliance.Kp = Kp 
            self.cartesian_compliance.Kr = Kr 
            
            self._command_int.x = x  # Tega ni v matlabu in tudi tu bom zakomentiral
            self._command_int.vel = xdot
            self._command_int.trq = trq
            
            # Publish
            #print(cmd_msg)

            self.cartesian_command_publisher.publish(cmd_msg)
            
            self.GetState()
            self.Update()
            
            while (tm() - self.last_control_time) < wait:
                0 #pass
            
            self.last_control_time= tm()
            
            # Hack
            tmperr = 0
            
            # Hack #2 - This should be set somewhero elso.
            #print("hit goto_x,", self.state.q)
            #self._command_int.q = self.state.q
            
            return tmperr
        else:
            raise Exception("%s: GoTo_x: Control strategy not supported"%self.Name)
        
    def GoTo_q(self, q, qdot, qddot=None, trq=0, wait=0.01, do_not_publish_msg = False, last_move=False):
        """GoTo_q Update q and qdot for joint controller and wait for time t
            
            Inputs:
            q       joint position (nj x 1)
            qdot    joint velocities position (nj x 1)
            trq     joint torques (nj x 1)
            wait    wait time after update
            do_not_publish_msg -   Control msg is not published. However the internal command parameters are set. This hack is used together with CMove, CMoveFor and JointTrapvelGoal 
        """

        if not self._control_strategy in ['JointPosition', 'JointVelocity', 'JointImpedance', 'JointPositionTrajectory', 'JointImpedanceAction']:
            raise Exception("GoTo_q: Invalid robot control_strategy!")
        
        if self._control_strategy in ['JointPosition', 'JointVelocity', 'JointImpedance']:
            # TODO : check if values are numeric (currently we only check if x, xdot are vectors)
            if not isvector(q, dim=7):
                raise Exception("%s : GoTo_q: NAN q value or wrong shape!"%self.Name)
            if not isvector(qdot, dim=7):
                raise Exception("%s : GoTo_q: NAN qdot value"%self.Name)
            if not isvector(trq, dim = 7):
                raise Exception("%s : GoTo_q: NAN trq value"%self.Name)
        
            #cmd_msg = rosmessage(robot.command_pub)
            cmd_msg = self.empty_JointCommand
            
            cmd_msg.pos = q
            cmd_msg.vel = qdot
            cmd_msg.trq = trq
            cmd_msg.impedance.n = 7
            cmd_msg.impedance.k = self.joint_compliance.K
            cmd_msg.impedance.d = self.joint_compliance.D
            
            if do_not_publish_msg == False:
                self.joint_command_publisher.publish(cmd_msg)
            
            #TODO: What is this????? 
            if math.floor(self._command_int.mode) == 1:
                [pp, RR, J] = self.kinmodel(q = q, out='pR')
                self._command_int.p = pp
                self._command_int.R = RR
                self._command_int.v = J*qdot
                
                #TODO: self._command_int.x = pp               
                
            self.GetState()
            self.Update()
            
            while (tm() - self.last_control_time < wait):
                pass
     
            self.last_control_time= tm()
                    
        if self._control_strategy in ['JointPositionTrajectory']:
            self.joint_trajectory_interface.add_point(positions=q, velocities=qdot, accelerations=qddot, delta_time=wait)    # q,qdot,qddot, self.tsamp)
            #self.GetState()
            #self.Update()
        
        if last_move:
            if self._control_strategy in ['JointPositionTrajectory']:     
                
                #time_for_calculations = tm() - self.joint_trajectory_start_time
                #rospy.loginfo("TIme for path calc: {}".format(time_for_calculations))
                
                self.joint_trajectory_interface.start()
                if self._capture_callback is not None:
                    start_t = tm()
                    last_record_time = tm()
                    while (tm() - start_t) < self.joint_trajectory_interface._time_sum:
                        cur_t = tm()
                        if (cur_t - last_record_time)> wait:
                            self.Update() # Record stuff if needed.
                            last_record_time = cur_t
                self.joint_trajectory_interface.wait(self.joint_trajectory_interface._time_sum)
                
                if len(self.recorded_joint_trajectories) > 5:
                    self.recorded_joint_trajectories= self.recorded_joint_trajectories[1:] # Delete the first recorded trajectory
                self.recorded_joint_trajectories.append(copy.deepcopy(self.joint_trajectory_interface._goal))
                #print(self.joint_trajectory_interface._goal)
            
            if self._control_strategy in ['JointImpedanceAction']:
                self.SendJointTrapVelGoal(q = q, **kwargs)
            
            #HACK: 
            [pp, RR, J] = self.kinmodel(q = q, out='pR')
            self._command_int.p = pp
            self._command_int.R = RR
            self._command_int.v = J*qdot
                
            self.GetState()
            self.Update()
            self.last_control_time=tm() 
            
            self._command_int.x = np.hstack((pp, r2q(RR))) # Dont delete this or else you have to set it somewhere-o-elso
            
        # Setting the internal values
        
        self._command_int.q = q
        self._command_int.qdot = qdot
        self._command_int.trq = trq
        
    def ApplyFT(self, FT):
        assert FT.shape[1] == 6
        cmd_msg = 0
        
        if self.control_strategy == 'CartesianImpedance':
            print("ApplyFT is unfinished, does nothing at all")
        else:
            print('%s : GoTo_X: Control strategy not supported'%self.Name)
        
    def error_recovery(self, enforce = False, reset_target = True):
        """ Recover Panda robot from error state (for example if emergency stop was pressed
        
        args :
        enforce : If true, all checks will be skipped and error recovery message will be sent. However the robot state will not be reset and the robot may make a jumping move""" 
        
        # Check if we are in an error state at all
        try:
            self.GetState()
            #print("Resetting current target")
            self.ResetCurrentTarget()
        except:
            pass
         # If controller has failed in the beginning, we cannot call GetState since state is not set at all. 
        if enforce==True:
            #print("Forcing error recovery")
            recovery_goal = ErrorRecoveryAction()
            self.error_recovery_action_client.send_goal(recovery_goal)
            sleep(0.1)
            return 0
                 
        controller_failed = 0
        if self.state is None:
            error_detected = 1
            controller_failed = 1
        else: 
            error_list = self.state.current_errors # We get an Errors object
            prev_error_list = self.state.last_motion_errors
            
            error_detected = 0
            #print(dir(error_list))
            attrs = dir(error_list)
            prev_attrs = dir(prev_error_list)
            
            for attr in attrs:
                # Attributes starting with _ are internal, for example __hash__. We dont care about them
                if attr[0] != '_':
                    value = getattr(error_list, attr)
                    if value is True:
                        error_detected = 1
                        rospy.loginfo("{} recovering from error - {}".format(self.Name, attr))
            
            if self.state.robot_mode in [fs.ROBOT_MODE_IDLE, fs.ROBOT_MODE_REFLEX, fs.ROBOT_MODE_OTHER]:
            #if self.state.robot_mode in [fs.ROBOT_MODE_REFLEX, fs.ROBOT_MODE_OTHER]:
                error_detected = 1
                rospy.loginfo("{} recovering from error because robot mode is {}".format(self.Name, self.state.robot_mode))
            if self.state.robot_mode == fs.ROBOT_MODE_USER_STOPPED:
                rospy.loginfo("Error recovery is not possible.\n{} User Stop pressed!".format(self.Name))
                raise Exception("Error recovery not possible, user stop is pressed!")
                return 0
        
        if error_detected:
            
            if (controller_failed == 0):
                self.GetState()
                self.ResetCurrentTarget()
                
            if self._control_strategy == 'CartesianImpedance':
                neutral_q = self.q_home
                self.SetCartImpContNullspace(q = neutral_q, k = np.zeros(7))
                
            recovery_goal = ErrorRecoveryAction()
            self.error_recovery_action_client.send_goal(recovery_goal);sleep(0.1)
        
            self.error_recovery_action_client.wait_for_result(rospy.Duration(3))
            print("{} got error recovery result. Waiting for robot mode to be {}".format(self.Name, fs.ROBOT_MODE_MOVE))
            
            # Wait for controller to be up and running.
            if self._control_strategy is not None:
                while not self.state.robot_mode not in  [fs.ROBOT_MODE_MOVE]:
                    pass
                print("{} robot mode is {}".format(self.Name, fs.ROBOT_MODE_MOVE))
        
        self.ACTIVE = False
        
        sleep(0.01)
            
        return 0
        
    def _load_controller(self, desired_controller):
        request = LoadControllerRequest()
        request.name = desired_controller
        try:
            #if self.load_controller_proxy.is_available(self.topic):
            success = self.load_controller_proxy.call(request)
            if success.ok == True:
                rospy.loginfo("Load {0}: Successful : {1}".format(request.name, success.ok))
                return 'continue'
            elif success.ok == False:
                rospy.loginfo("Load {0}: Successful :{1}".format(request.name, success.ok))
                return 'failed'
            #else:
            #     rospy.loginfo("Service is not available!")
            #     return 'failed'
        except Exception as e:
            rospy.loginfo(e)
            return 'failed'
        
    def _unload_controller(self, desired_controller):
        request = UnloadControllerRequest()
        request.name = desired_controller
        try:
            #if self.load_controller_proxy.is_available(self.topic):
            success = self.unload_controller_proxy.call(request)
            if success.ok == True:
                rospy.loginfo("Unload  {0}: Successful : {1}".format(request.name, success.ok))
                return 'continue'
            elif success.ok == False:
                rospy.loginfo("Unload {1}: Successful :{1}".format(request.name, success.ok))
                return 'failed'
            #else:
            #     rospy.loginfo("Service is not available!")
            #     return 'failed'
        except Exception as e:
            rospy.loginfo(e)
            return 'failed'
        
    def _switch_controller(self, start_controller, stop_controller, strictness):
        """
        Parameters as follow

        -- start_controller    string[]   start_controller_array
        -- stop_controller     string[]   stop_controller_array
        -- strictness          int32"""
        self.ACTIVE = False # Make sure we send the activate message later ! 
        
        request = SwitchControllerRequest()
        request.start_controllers = start_controller
        request.stop_controllers = stop_controller
        request.strictness = strictness
        #rospy.loginfo("request: {0}".format(request))
        
        if len(start_controller) == 0:
            0
        elif start_controller[0] == '':
            #We want to keep the last control strategy in case we want to just start the controller again.
            0
            #self._control_strategy = None
        elif start_controller[0] not in self.controller_names.values():
            rospy.loginfo("Unsupported controller type:{0}".format(start_controller[0]))
        else:
            self._control_strategy = self.controller_names_inv[start_controller[0]]
        
        #if start_controller[0] == 'joint_impedance_controller':
        #    self._control_strategy = 'JointImpedance'
        #elif start_controller[0] == 'cartesian_impedance_controller':
        #    self._control_strategy = 'CartesianImpedance'
        #elif start_controller[0] == 'joint_position_controller':
        #    self._control_strategy = 'JointTrajectory'

        #if self.switch_controller_proxy.is_available(self.switch_controller_topic):
        success = self.switch_controller_proxy.call(request)
        if success.ok == True:
            rospy.loginfo("Switch to {0}. Stop {1}. Successful: {2}".format(request.start_controllers, request.stop_controllers[0], success.ok))
            return 'continue'
        elif success.ok == False:
            rospy.loginfo("Switch to {0}. Stop {1}. Successful: {2}".format(request.start_controllers, request.stop_controllers[0], success.ok))
            return 'failed'
        #else:
            #rospy.loginfo("Service is not available!")
            #return 'failed'
            
    def Unload_controller(self):
        """Unloads the currently running controller """
        c = self.controller_names[self._control_strategy]
        self.Stop_controller()
        self._unload_controller(desired_controller = c)
        rospy.loginfo("Unloading controller {0}".format(c))
        
    def Load_controller(self):
        """ Loads the controller currently selecter by self._control_strategy"""
        c = self.controller_names[self._control_strategy]
        self._load_controller(desired_controller = c)
        rospy.loginfo("Loading controller {0}".format(c))
        
    def Stop_controller(self):
        
        c = [self.controller_names[self._control_strategy]]
        
        self._switch_controller(start_controller = [''], stop_controller = c, strictness = 1 )

    def Start_controller(self):
            
        c = [self.controller_names[self._control_strategy]]
        
        self._switch_controller(start_controller = c, stop_controller = [''], strictness = 1)

    def Switch_controller(self, start_controller = None):
        
        assert start_controller in self.controller_names.keys()
        
        if self._control_strategy == start_controller:
            rospy.loginfo("Not switching because already using {}".format(start_controller))
            
            return 0
        
        start_controller = [self.controller_names[start_controller]]
        
        stop_controllers = [controller for controller in self.controller_names_inv.keys() if controller != start_controller[0]]
        #stop_controller = [self.controller_names[self._control_strategy]]
        
        #assert start_controller != stop_controller
        # Reset the flags.
        self.Stop()
        
        #HACK:
        self.ACTIVE = False
        
        self._switch_controller(start_controller = start_controller, stop_controller= stop_controllers, strictness = 1)
        return 1
        
    def SetReconcycleCartesianCompliance(self, Kp = None, Kr= None, R = None, D = None, hold_pose = 'on'):
        """ Sets the cartesian compliance, but ONLY for Reconcycle cartesian impedance controller"""
        rospy.loginfo("Setting cart compl")
        if Kp is None:
            Kp = self.cartesian_compliance.Kp
        if Kr is None:
            Kr = self.cartesian_compliance.Kr
        if R is None:
            R = self.cartesian_compliance.R 
        if D is None:
            D = self.cartesian_compliance.D
        
        assert len(Kp) == 3
        assert len(Kr) == 3
        for i in Kp: assert i>= 0
        for i in Kr: assert i>=0
        
        assert hold_pose in ['on', 'off']
        
        if self._control_strategy != 'CartesianImpedance':
            print("Setting reconcycle cartesian compliance, but we are not in cartesian impedance control strategy")
            return 1
        
        # Calculate stiff matrix
        trM = np.diag(Kp)
        rotM = np.diag(Kr)

        # Rotate
        trK = R * trM * np.transpose(R)
        rotM = np.diag(Kr)

        # Damping
        trD = R*2*np.sqrt(trM)*np.transpose(R)
        rotD = D*np.sqrt(rotM)

        # Check for NaN
        try:
            assert not np.isnan(trD).any()
            assert not np.isnan(rotD).any()
            assert not np.isnan(trK).any()
            assert not np.isnan(rotM).any()
        except:
            rospy.loginfo("SetCartesianCompliance NaNs present in trD or rotD or trK or rotK.")
            rospy.loginfo("trD: {0}".format(trD))
            rospy.loginfo("rotD: {0}".format(rotD))
            rospy.loginfo("trK: {0}".format(trK))
            rospy.loginfo("rotM: {0}".format(rotM))
            return

        stiffness = ImpedanceParameters()
        stiffness.n = 9
        stiffness.k = np.concatenate((np.reshape(trK, (9,1)), np.reshape(rotM,(9,1))))
        stiffness.d = np.concatenate((np.reshape(trD, (9,1)), np.reshape(rotD, (9,1))))

        if hold_pose == 'off':
            self.cart_stiff_publisher.publish(stiffness)
        else:
            cmd_msg = CartesianCommand()
            cmd_msg.impedance = stiffness
            
            # Reset current robot target
            #stiffness = ImpedanceParameters()
            self.ResetCurrentTarget()
            #self.cart_stiff_publisher.publish(stiffness)
            self.GetState()
            
            #rospy.loginfo("{}".format(self._actual_int.x))
            #rospy.loginfo("{}".format(self._command_int.x))

            cmd_msg.pose.position.x = self._actual_int.x[0]
            cmd_msg.pose.position.y = self._actual_int.x[1]
            cmd_msg.pose.position.z = self._actual_int.x[2]
            
            cmd_msg.pose.orientation.w = self._actual_int.x[3]
            cmd_msg.pose.orientation.x = self._actual_int.x[4]
            cmd_msg.pose.orientation.y = self._actual_int.x[5]
            cmd_msg.pose.orientation.z = self._actual_int.x[6]
            
            cmd_msg.time_from_start = rospy.Duration(0,0)
            
            # Send command message
            self.cartesian_command_publisher.publish(cmd_msg)
             
        # Handle (update) internal cartesian impedance values 
        self.cartesian_compliance.Kp = Kp
        self.cartesian_compliance.Kr = Kr
        self.cartesian_compliance.R = R
        self.cartesian_compliance.D = D
                        
        return 0
    
    def SetCartesianImpedanceFranka(self, stiffness):
        """Sets the cartesian impedance of the internal franka controller. Generally, this should be set to high values, and then the actual stiffness
        should be set in the Reconycle controller"""
        request = SetCartesianImpedance()
        request.cartesian_stiffness = stiffness
        try:
            if 1:
            #if self.K_frame_proxy.is_available(self.K_frame_topic):
                success = self.cart_impedance_franka_proxy.call(request)
                if success.success == True:
                    rospy.loginfo("Franka cart imp. successful: {0}".format(success.success))
                    return 0
                elif success.success == False:
                    rospy.loginfo("Franka cart imp. successful: {0}".format(success.success))
                    return 1
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo(e)
            return 1
        return 0
        
    def SetKFrame(self, EE_T_K):
        '''Sets the transformation \(^{EE}T_K\) from end effector frame to stiffness frame.
        The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    
        Parameters as follow
        -- EE_T_K           float[16]   Vectorized EE-to-K transformation matrix , column-major.
        '''
        request = SetKFrameRequest()
        request.EE_T_K = EE_T_K
        try:
            if 1:
            #if self.K_frame_proxy.is_available(self.K_frame_topic):
                success = self.K_frame_proxy.call(request)
                if success.success == True:
                    rospy.loginfo("Successful: {0}".format(success.success))
                    return 0
                elif success.success == False:
                    rospy.loginfo("Successful: {0}".format(success.success))
                    return 1
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo(e)
            return 1
        return 0
    
    def SetEEFrame(self, NE_T_EE):
        '''
        Sets the transformation \(^{NE}T_{EE}\) from nominal end effector to end effector frame.
        The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    
        Parameters as follow
        -- NE_T_EE          float[16]   4x4 matrix -> Vectorized NE-to-EE transformation matrix , column-major.
        '''
        request = SetEEFrameRequest()
        request.NE_T_EE = NE_T_EE

        try:
            if 1:
            #if self.EE_frame_proxy.is_available(self.K_frame_topic):
                success = self.EE_frame_proxy.call(request)
                if success.success == True:
                    rospy.loginfo("Successful: {0}".format(success.success))
                    return 0
                elif success.success == False:
                    rospy.loginfo("Successful: {0}".format(success.success))
                    return 1
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo(e)
            return 1
        return 0
    
    def shutdown_hook(self):
        rospy.loginfo("{0} shutting down".format(self.Name))
    def on_shutdown(self):
        rospy.on_shutdown(self.shutdown_hook)
    
    def SetFullCollisionBehavior(self, F_acc, F_nominal = None, T_acc= None, T_nominal= None ):
        """ TODO ! """
        """ SetCollisionBehavior Set nominal collision thresholds
        
            % Input:
            %   F       collision task force treshold [N]
            %   T       collision task torques treshold [Nm]
            %   tq      collision joint torque treshold [Nm]
        """
        if F_nominal is None:
            F_nominal = 0.5* F_acc
        
        if T_acc is None:
            T_acc = np.ones(7)*F_acc
        else:
            T_acc = np.ones(7)*T_acc
            
        if T_nominal is None:
            T_nominal = np.ones(7)*F_nominal
        else:
            T_nominal = np.ones(7)*T_nominal
        
        F_acc = np.ones(6)*F_acc
        F_nominal = np.ones(6)*F_nominal
        
        # Stop controller
        self.Stop_controller()
        
        collision_behavior = SetFullCollisionBehaviorRequest()
        # Upper limits
        collision_behavior.upper_force_thresholds_acceleration = F_acc
        collision_behavior.upper_force_thresholds_nominal = F_nominal
        
        collision_behavior.upper_torque_thresholds_acceleration = T_acc
        collision_behavior.upper_torque_thresholds_nominal = T_nominal
        
        # Lower limits 
        collision_behavior.lower_force_thresholds_acceleration = F_acc  * 0.5
        collision_behavior.lower_force_thresholds_nominal = F_nominal *0.5
        
        collision_behavior.lower_torque_thresholds_acceleration = T_acc *0.5
        collision_behavior.lower_torque_thresholds_nominal = T_nominal*0.5

        #call(robot.set_force_torque_collision_behavior,collision_behavior,'Timeout',2);
        #rospy.loginfo("Calling ")
        #self.force_torque_limits_proxy.call(collision_behavior)
        
        try:
            if 1:
            #if self.force_torque_limits_proxy.is_available(self.force_torque_limits_topic, timeout=2):
                success = self.full_force_torque_limits_proxy.call(collision_behavior)
                if success.success == True:
                    rospy.loginfo("FullColThr call: {0}".format(success.success))
                    rospy.loginfo('FullCollision behavior changed: \nF_acc   [N]: {0} \nF_nom [N]: {1} \nT_acc [Nm]:{2}\nT_nom [Nm]:{3}'.format(F_acc,F_nominal,T_acc,T_nominal))
                elif success.success == False:
                    rospy.loginfo("Successful: {0}".format(success.success))
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo("FullCollisionThresholds exception: {}".format(e))
            return 1        
        
        sleep(0.3)
        self.Start_controller()
        
    def SetCollisionBehavior(self,F, T = None,tq = None):
        """ SetCollisionBehavior Set nominal collision thresholds
        
            % Input:
            %   F       collision task force treshold [N]
            %   T       collision task torques treshold [Nm]
            %   tq      collision joint torque treshold [Nm]
        """
        
        assert F >0
        assert type(F) in [float, int]
        
        if T is None:
            T = np.ones(3)*F
        else:
            T = np.ones(3)*T
        
        if tq is None:
            tq = np.ones(7)*F
        else:
            tq = np.ones(7)* tq
            
        F = np.ones(3)*F
        
        # Stop controller
        self.Stop_controller()
        collision_behavior = SetForceTorqueCollisionBehaviorRequest()
        
        collision_behavior.upper_force_thresholds_nominal = np.concatenate((F,T),axis =None)
        collision_behavior.lower_force_thresholds_nominal = np.concatenate((F,T),axis=None)*0.5
        collision_behavior.lower_torque_thresholds_nominal = tq*0.5
        collision_behavior.upper_torque_thresholds_nominal = tq
        
        self.collision_thresholds.F=F[0]
        self.collision_thresholds.T=T[0]
        self.collision_thresholds.tq=tq
            
        #call(robot.set_force_torque_collision_behavior,collision_behavior,'Timeout',2);
        #rospy.loginfo("Calling ")
        #self.force_torque_limits_proxy.call(collision_behavior)
        
        try:
            if 1:
            #if self.force_torque_limits_proxy.is_available(self.force_torque_limits_topic, timeout=2):
                success = self.force_torque_limits_proxy.call(collision_behavior)
                if success.success == True:
                    rospy.loginfo("ColThr call: {0}".format(success.success))
                    rospy.loginfo('Collision behavior changed: \nF   [N]: {0} \nT [Nm]: {1} \nJoint T [Nm]:{2}'.format(F,T,tq))
                elif success.success == False:
                    rospy.loginfo("Successful: {0}".format(success.success))
        except Exception as e:
        #except (CommandException, NetworkException) as e :
            rospy.loginfo("ColThr exception: {}".format(e))
            return 1        
        
        sleep(0.3)
        self.Start_controller()
        
    def SendJointTrapVelGoal(self,q,max_vel=0.7,max_acc=1, **kwargs):
        # Use action client of the controler to move the robot to desired configuration q
        #print(max_vel, max_acc)
        joint = JointState()
        joint.position = q
        goal = JointTrapVelGoal([joint], max_vel, max_acc)
        #print("Goal sent")
        self.joint_action_client.send_goal(goal)
        #self.joint_action_client.wait_for_result(timeout = rospy.Duration(10))
        self.joint_action_client.wait_for_result()
        
        #print("Got success response")
        #self.GetState() # Update q.s
        #HACK: update commanded qs
        self.GoTo_q(q=q, wait = 0.1, qdot = np.zeros(7), trq = np.zeros(7), do_not_publish_msg = True)
    
    def SetLoggerLevel(self, level):
        assert level in ['debug', 'info']
        log_msg = SetLoggerLevelRequest()
        #log_msg.logger = 'panda_action_server'
        log_msg.logger = 'ros.ijs_controllers'
        log_msg.level = level
        self.logger_svc_publisher.call(log_msg)
        
        if level == 'debug':self.verbose = 3
        elif level == 'info': self.verbose=1
        
    def GetStiffnessFrame(self):
        self.GetState()
        Tx = self.state.EE_T_K
        T = np.reshape(Tx, (4,4))
        print("Stiffness frame EE_T_K:")
        print(T)
    
    def SetStiffnessFrame(self, T = None):
        """  SetStiffnessFrame Sets the stiffness frame (EETK) relative to EE frame
            (controller is temporary stopped!) """
        if T is None:
            newT = np.eye(4)
        else:
            0
        print("SetStiffnessFrame UNFINISHED")
        
    def AvailableStrategies(self):
        strategies = ['JointPosition', 'JointPositionTrajectory', 'JointImpedance','CartesianVelocity','CartesianImpedance']
        print(strategies)
        return strategies
    
    def AvailableInternalStrategies(self):
        strategies = ['JointImpedance','CartesianImpedance']
        print(strategies)
        return strategies
    
    def rotate_by_rpy(self, d_rpy = [0,0,0]):
        """ Function takes in delta rpy (how much we want to rotate the gripper) in degrees and returns the new T matrix. """ 
        self.GetState()
        rpy = r2rpy(self.R, out ='deg')
        rpy += d_rpy
        R_new = rpy2r(rpy, out = 'R', unit ='deg' )
        T_new = self.T.copy()
        T_new[0:3, 0:3] = R_new
        
        return T_new
    
    def check_contact(self):
        # Checks if a contact is ocurring (which contact level is activated).
        # if contact_values are all zero, there is no contact
 
        # Note : if you check for cartesian_COLLISION, when it happens, the reflex is activated and
        # after that the controller must be restarted
        contact_values = self.state.cartesian_contact
        for v in contact_values:
            if v > 0:
                print("panda_ros.py - contact detected") 
                return 1
        # If we get to here, no collisions are detected
        return 0 

    def Start(self):
        if not self.ACTIVE and (self._control_strategy in ['CartesianImpedance']):
            self.Activate_controller()
            self.ACTIVE = True
        if self._control_strategy in ['JointPositionTrajectory']:
            #rospy.loginfo("Start() running")
            self.joint_trajectory_interface.clear()
            self.joint_trajectory_start_time = tm() # Keep this for logging how long calculations take.

        #return super(panda_ros, self).CMove(x,t,traj,wait,FT,**kwargs)
        return robot.Start(self)
    
    def Stop(self):
                
        return robot.Stop(self)
    
    def reset_recorded_states(self):
        self.recorded_states_list = []
        
    def record_states(self, r = None):
        """
        r - robot_obj (panda_ros)"""
        if r is None:
            e = [self.state, copy.deepcopy(self._actual_int), copy.deepcopy(self._command_int)]
        else:
            e = [r.state, copy.deepcopy(r._actual_int), copy.deepcopy(r._command_int)]
        self.recorded_states_list.append(e)
        #self.recording_n +=1


def record_movement_in_while_loop(robot_obj, t):
    r = robot_obj
    tsamp = robot_obj.tsamp
    t_record = []
    
    r.reset_recorded_states()

    t0 = tm()
    t_record = t
    
    while (tm() - t0) < t_record:
        r.GetState()
        r.record_states()
        sleep(tsamp)
        
        o = np.array(r.recorded_states_list)
    xi = []
    qi = []
    for i in range(0, o[:,1].shape[0]):
        xi.append(o[i,1].x)
        qi.append(o[i,1].q)
        
    return xi, qi

def execute_arbitrary_joint_trajectory(robot_obj, qs, qdots, times):
    
    r = robot_obj
    r.error_recovery()
    
    # Safety checks
    assert qs.shape[1] == r.nj
    assert qdots.shape[1] == r.nj
    
    if r._control_strategy != 'JointPositionTrajectory':
        r.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        
    # If we are not close to the initial position, perform a slow JMove towards it. REALLY SLOW. The robot should be in the desired position already.
    r.GetState();
    cur_q = np.array(copy.deepcopy(r._actual_int.q))
    if np.sum(np.abs(cur_q - qs[0,:])) > 0.05 :
        r.JMove(qs[0,:], 4, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
     
    r.safety_check() # Check robot is still and stuff.
     
    trq = np.zeros(r.nj)
    r.Start()

    t_move = np.diff(times)

    r.GoTo_q(q = qs[0,:], qdot = np.zeros(r.nj), qddot = None, trq = trq, wait = 0)
    for qt, qdt, wait_t in zip(qs,qdots, t_move):
        if r._do_motion_check and r._motion_check_callback is not None:
            tmperr = r._motion_check_callback(r)
            if tmperr>0:
                r.WarningMessage('Motion aborted')
                raise Exception('motion aborted')#return tmperr
                        
        # Timer nastavi na nulo. Potem tukej glej ali je ta dt res tak kot tsamp. Ce je premajhen ga pocakas. Nastavis last command time in gledas razlike.
        r.GoTo_q(q = qt, qdot = qdt, qddot = None, trq = trq, wait = wait_t)
    r.GoTo_q(q = qs[-1,:], qdot = np.zeros(r.nj),qddot = None, trq = trq, wait = wait_t, last_move = True)

    r.Stop()
    
    return 0

def execute_arbitrary_cartesian_trajectory(robot_obj, xs, xdots, times):
    
    r = robot_obj
    
    # Safety checks
    assert xs.shape[1] == r.nj
    assert xdots.shape[1] == 6
    
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller =  'CartesianImpedance')
    r.error_recovery()
    
    # If we are not close to the initial position, perform a slow JMove towards it. REALLY SLOW. The robot should be in the desired position already.
    r.GetState();
    cur_x = np.array(copy.deepcopy(r._actual_int.x))
    if np.sum(np.abs(cur_x[0:3] - xs[0,0:3])) > 0.05 :
        r.CMove(xs[0], 3, v_max_factor = 0.35, a_max_factor = 0.35)
    #    r.JMove(qs[0,:], 4, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
     
    r.safety_check() # Check robot is still and stuff.
     
    trq = np.zeros(6)
    r.Start()

    r.GoTo_X(x = xs[0,:], xdot = np.zeros(6), xddot = None, trq = trq, wait = r.tsamp)
    for xt, xdt in zip(xs,xdots):
        if r._do_motion_check and r._motion_check_callback is not None:
            tmperr = r._motion_check_callback(r)
            if tmperr>0:
                r.WarningMessage('Motion aborted')
                raise Exception('motion aborted')#return tmperr
                        
        # Timer nastavi na nulo. Potem tukej glej ali je ta dt res tak kot tsamp. Ce je premajhen ga pocakas. Nastavis last command time in gledas razlike.
        r.GoTo_X(x = xt, xdot = xdt, xddot = None, trq = trq, wait = r.tsamp)
    r.GoTo_X(x = xs[-1,:], xdot = np.zeros(6),xddot = None, trq = trq, wait = r.tsamp, last_move = True)

    r.Stop()
    
    return 0
