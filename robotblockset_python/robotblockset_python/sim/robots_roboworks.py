import numpy as np
from time import perf_counter, sleep

from transformations import rp2t
from roboworks import RoboWorksScene
from robot_spec import panda, lwr, ur10, ur5
from robots import robot, robot_no_compliance

class robot_roboworks(robot):
    def __init__(self,scn, host='127.0.0.1', sock=None, **kwargs):
        robot.__init__(self, **kwargs)
        self.Name = self.Name+'RoboWorks:'
        self.BaseTagNames = 'x,y,z,roll,pitch,yaw'
        self.TCPTagNames = 'tcpx,tcpy,tcpz,tcproll,tcppitch,tcpyaw'
        self.JointTagNames = ''
        for i in range(self.nj):
            self.JointTagNames += 'q'+str(i+1)+','
        self.JointTagNames = self.JointTagNames[:-1]    
        self.host = '127.0.0.1'
        self.scene = RoboWorksScene(sock=sock)
        if self.scene.connect(scn,host):
            print(f'Robot connected to scene: {scn}')
        self.tsamp = 0.01        # sampling rate 
        self._control_strategy = 'JointPosition'
    
    def __del__(self):
        self.scene.disconnect()
        self.Message('RoboWorks scene disconnected')   

    def Close(self):
        self.scene.disconnect()
        self.Message('RoboWorks scene disconnected')   

    def Init(self):
        self._t0=self.simtime()
        pose = self.scene.get_tag_values(self.BaseTagNames)
        self.TBase=rp2t(RPY=(pose[3:6]),p=pose[:3], out='T')
        pose = self.scene.get_tag_values(self.TCPTagNames)
        self.TCP=rp2t(RPY=(pose[3:6]),p=pose[:3], out='T')
        self.Message('Init',3)
        self.GetState()
        self._command.q = self._actual.q
        self._command.qdot = self._actual.qdot
        self._command.x = self._actual.x
        self._command.v = self._actual.v
        self.ResetTime()
        
    def GetState(self):
        if (self.simtime()-self._last_update)>self.tsamp:
            qq = self.scene.get_tag_values(self.JointTagNames)
            if qq is not None:
                self._actual.q = np.array(qq)
            self._actual.qdot = np.zeros(self.nj)
            q = self._actual.q
            x = self.kinmodel(q)
            self._actual.x = x[0]
            self._actual.v = np.zeros(6)
            self._actual.FT = np.zeros(6)
            self._actual.trq = np.zeros(self.nj)
            self._actual.trqExt = np.zeros(self.nj,)
            self._tt=self.simtime()
            self._last_update = self.simtime()

    def GoTo_q(self, q, qdot, trq, wait):
        """Update joint positions and wait

        Parameters
        ----------
        q : array of floats
            desired joint positions (nj, )
        qdot : array of floats
            desired joint velocities (nj, )
        trq : array of floats
            additional joint torques (nj, )
        wait : float
            Maximal wait time since last update. Defaults to 0.
        """
        self.sinhro_control(wait)
        self._last_control_time = self.simtime()
        self.scene.set_tag_values(self.JointTagNames, q)
        self._command.q = q
        self._command.qdot = qdot
        self._command.trq = trq
        x, J = self.kinmodel(q)
        self._command.x = self.BaseToWorld(x)
        self._command.v = self.BaseToWorld(J@qdot)
        self.Update()

    def SetStrategy(self, strategy):
        pass

    def get_val(self, tagNames):
        return self.scene.get_tag_values(tagNames)

    def set_val(self, tagNames, tagValues):
        self.scene.set_tag_values(tagNames, tagValues)

class panda_roboworks(robot_roboworks, robot_no_compliance, panda):
    def __init__(self, scn='Panda', **kwargs):
        panda.__init__(self)
        robot_roboworks.__init__(self, scn, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_roboworks.__del__(self)
        self.Message("Robot deleted",2)

class lwr_roboworks(robot_roboworks, robot_no_compliance, lwr):
    def __init__(self, scn='LWR', **kwargs):
        lwr.__init__(self)
        robot_roboworks.__init__(self, scn, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_roboworks.__del__(self)
        self.Message("Robot deleted",2)

class ur10_roboworks(robot_roboworks, robot_no_compliance, ur10):
    def __init__(self, scn='UR10', **kwargs):
        ur10.__init__(self)
        robot_roboworks.__init__(self, scn, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_roboworks.__del__(self)
        self.Message("Robot deleted",2)

class ur5_Roboworks(robot_roboworks, robot_no_compliance, ur5):
    def __init__(self, scn='UR5', **kwargs):
        ur5.__init__(self)
        robot_roboworks.__init__(self, scn, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_roboworks.__del__(self)
        self.Message("Robot deleted",2)


if __name__ == '__main__':
    from roboworks import isRoboWorks
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    r = panda_roboworks()
    print('Roboworks ? ', str(isRoboWorks(r.scene)))
    mod = r.kinmodel(out='pR')
    print('Robot kinematics:\n ',mod)
    print('Task pose: ',r.GetPose(task_space='World'))
    print('Base pose: ',r.GetPose(task_space='Robot'))
    r.GetState()
    print('Time 1: %.3f'%r.Time, r.q)
    r.JMove(r.q_home,1)
    print('Time 2: %.3f'%r.Time, r.q)
    


