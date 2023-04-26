from abc import ABC, abstractmethod, ABCMeta
from time import sleep

from robotblockset_python.sim.roboworks import isRoboWorks

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import robot_module_msgs.msg
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import time

class Gripper(metaclass=ABCMeta):
    def __init__(self, **kwargs):
        self.Name = 'Gripper:'
        self._verbose = 2        # verbose level
        self._state = -1         # gripper state
        self._error = False
        self._robot = None       # robot to which gripper is attached

    @abstractmethod
    def open(self, **kwargs):
        pass

    @abstractmethod
    def close(self, **kwargs):
        pass

    @abstractmethod
    def grasp(self, **kwargs):
        pass

    def isOpened(self):
        return self._state==0

    def isClosed(self):
        return self._state==1

    def getState(self):
        if self._state==0:
            return 'Opened'
        elif self._state==1:
            return 'Closed'
        else:
            return 'Undefined'    

    def attachTo(self, robot):
        self._robot = robot
    
    def Message(self, msg, verb=0):
        if self._verbose>0 and self._verbose>=verb:
            print(f'{self.Name}:{msg}')

    def WarningMessage(self, msg):
         print(f'Warning: {self.Name}:{msg}')

#class QbRoboticsGripper(Gripper):
#    def __init__(self, **kwargs):
#        self.actionGoal = FollowJointTrajectoryGoal()
        
         
class VariableStiffnessGripper(Gripper):
    def __init__(self, **kwargs):
        """ QbRobotics Variable Stiffness (VS) gripper"""
        #super().__init__()
        self.Name = 'vsgripper'
        
        # Time to complete movement.
        self.motion_duration = 1
        self.sleep_duration = 1.2*self.motion_duration

        self._topic = '/qbsoftclaw/control/qbsoftclaw_position_and_preset_trajectory_controller/follow_joint_trajectory'
        self._client = actionlib.SimpleActionClient(self._topic, FollowJointTrajectoryAction)   # ProxyActionClient({self._topic: FollowJointTrajectoryAction})
        self.actionGoal = FollowJointTrajectoryGoal()
        self.actionGoal.trajectory.joint_names = ['qbsoftclaw_shaft_joint', 'qbsoftclaw_stiffness_preset_virtual_joint']
        
        self.actionPoint = JointTrajectoryPoint() # Init empty
        self.actionPoint.positions = [0,0]
        self.actionPoint.velocities = [0,0]
        self.actionPoint.accelerations = [0,0]
        self.actionPoint.effort = [0,0]
        self.actionPoint.time_from_start = rospy.Duration(self.motion_duration) # seconds
        
        rospy.loginfo("SoftClaw DEFAULT close command is 0.9 so we don't burn out the motors. Set it to 1 for stronger grip.")
        
    def open(self, command = 1, stiffness = 0.9, sleep=False, wait_for_result = False, **kwargs):
        """ If input command == 1, then  it opens to the max. if 0, it will close. if in between...
        Args :
        command 
        sleep - if true, sleep for the required time to open/close gripper
        wait_for_result: wait for result from the action server""" 
        
        assert (0 <= command <= 1)
        assert (0 <= stiffness <= 1)

        # Command must be between 0 and 1. By default it is one
        
        actualcommand = -command  # If actualcommand is -1, gripper will open to the max
        
        self.actionPoint.positions = [actualcommand, stiffness] 
        self.sendActionGoal(wait_for_result)
        if sleep:time.sleep(self.sleep_duration)

    def close(self, command = 0.9, stiffness = 0.9, sleep=False, wait_for_result= False, **kwargs):
        """Args :
        command 
        sleep - if true, sleep for the required time to open/close gripper
        wait_for_result: wait for result from the action server"""
        
        assert (0 <= command <= 1)
        assert (0 <= stiffness <= 1)
        
        actualcommand = command - 1

        self.actionPoint.positions = [actualcommand, stiffness] 
        self.sendActionGoal(wait_for_result)
        if sleep:time.sleep(self.sleep_duration)
    
    def sendActionGoal(self, wait_for_result = False):
        
        self.actionGoal.trajectory.points = [self.actionPoint]
        
        #rospy.loginfo("Starting sending softclaw goal...")
        try:
            
            rospy.loginfo("{} goal sent: {}".format(self.Name, str(self.actionPoint.positions)))
            self._client.send_goal(self.actionGoal)
            if wait_for_result:
                res = self._client.get_result(self._topic)
                while res == None:
                    res = self._client.get_result(self._topic)
                    time.sleep(0.2)
                rospy.loginfo("{} result: {}".format(self.Name, res))
            self._state = 0

        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            rospy.loginfo('{} Failed to send the goal command:\n{}'.format(self.Name, str(e)))
            self._error = True

    def grasp(self, **kwargs):
        self.Close()

class SofthandGripper(Gripper):
    def __init__(self, **kwargs):
        """ QbRobotics SoftHand gripper"""
        #super().__init__()
        
        
        self.Name = 'softhand'
        
        # Time to complete movement.
        self.motion_duration = 0.7
        self.sleep_duration = 1.2*self.motion_duration
        
        self._topic = '/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory'
        self._client = actionlib.SimpleActionClient(self._topic, FollowJointTrajectoryAction)   # ProxyActionClient({self._topic: FollowJointTrajectoryAction})
        self.actionGoal = FollowJointTrajectoryGoal()
        self.actionGoal.trajectory.joint_names = ['qbhand1_synergy_joint']
       
        self.actionPoint = JointTrajectoryPoint()
        self.actionPoint.time_from_start = rospy.Duration(self.motion_duration) # seconds
        
    def open(self, command = 1, sleep = False, wait_for_result = False, **kwargs):
        """ If input command == 1, then  it opens to the max. if 0, it will close. if in between...
        Args :
        command 
        sleep - if true, sleep for the required time to open/close gripper
        wait_for_result: wait for result from the action server""" 
        
        assert (0 <= command <= 1)
        # Command must be between 0 and 1. By default it is one
        
        actualcommand = 1-command  # If actualcommand is 0, gripper will open to the max. If 1, it will close. (Thats how softhand actions work).
        
        self.actionPoint.positions = [actualcommand] 
        self.sendActionGoal(wait_for_result)
        if sleep:time.sleep(self.sleep_duration)

    def close(self, command = 1, sleep = False, wait_for_result = False, **kwargs):
        """Args :
        command 
        sleep - if true, sleep for the required time to open/close gripper
        wait_for_result: wait for result from the action server"""
        assert (0 <= command <= 1)
        
        actualcommand = command

        self.actionPoint.positions = [actualcommand] 
        self.sendActionGoal(wait_for_result)
        if sleep:time.sleep(self.sleep_duration)
    
    def sendActionGoal(self, wait_for_result = False):
        
        self.actionGoal.trajectory.points = [self.actionPoint]
        
        #rospy.loginfo("Starting sending softhand goal...")
        try:
            
            rospy.loginfo("{} goal sent: {}".format(self.Name, str(self.actionPoint.positions)))
            self._client.send_goal(self.actionGoal)
            if wait_for_result:
                res = self._client.get_result(self._topic)
                while res == None:
                    res = self._client.get_result(self._topic)
                    time.sleep(0.2)
                rospy.loginfo("{} result: {}".format(self.Name, res))

            self._state = 0

        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            rospy.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True

    def grasp(self, **kwargs):
        self.Close()
         
class dummygripper(Gripper):
    def __init__(self, **kwargs):
        gripper.__init__(self, **kwargs )
        self.Name = 'DummyGripper:'
        self._state = -1

    def open(self, **kwargs):
        self._state = 0
        return 1

    def close(self, **kwargs):
        self._state = 1
        return 1

    def Grasp(self, **kwargs):
        self._state = 0
        return 1

    def Move(self, width, **kwargs):
        self._state = -1
        return 1

    def Homing(self, **kwargs):
        self._state = 0
        return 1

class panda_gripper_roboworks(Gripper):
    def __init__(self, **kwargs):
        gripper.__init__(self, **kwargs )
        self.Name = 'Panda:Gripper:RobotWorks:'
        self.GripperTagNames = 'gripper'
        self._width_grasp = 0
        self._width = 0
        self._width_max = 0.077
        self._state = -1

    def open(self, **kwargs):
        self.Move(self._width_max)
        self._state = 0
        return 1

    def close(self, **kwargs):
        self.Move(0)
        self._state = 1
        return 1

    def Grasp(self, **kwargs):
        kwargs.setdefault('width',0)
        self.Move(max(kwargs['width'],self._width_max))
        self._state = 1
        return 1

    def Move(self, width, **kwargs):
        self._width = min(width, self._width_max)
        if self._robot is not None and isRoboWorks(self._robot.scene):
            self._robot.set_val(self.GripperTagNames,self._width)
        sleep(1)
        self._state = -1
        return 1

    def Homing(self, **kwargs):
        return self.Open

def isgripper(obj):
    return isinstance(obj, gripper)
