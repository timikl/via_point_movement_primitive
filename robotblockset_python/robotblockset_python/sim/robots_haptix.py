import numpy as np
from time import perf_counter, sleep
import ctypes
c_float_p = ctypes.POINTER(ctypes.c_float)

from validateargs import isvector
from transformations import map_pose, r2q, checkQ, checkx
import mujoco_haptix as haptix
from mujoco_haptix import isHaptix
from robot_spec import panda, lwr, ur10, ur5
from robots import robot, robot_no_compliance

class robot_haptix(robot):
    def __init__(self, robot_name, dll_path=None, host=None, **kwargs):
        robot.__init__(self, **kwargs)
        self.Name = self.Name+'Haptix:'
        self._BaseName = robot_name
        self._control_strategy = 'JointPosition'
        self._JointNames = []
        self._ActuatorNames = []
        for i in range(self.nj):
            self._JointNames.append(self._BaseName+'/joint'+str(i+1))
            self._ActuatorNames.append(self._BaseName+'/pos_joint'+str(i+1))
        self._EEName = self._BaseName+'/EE'
        self._TCPName = self._BaseName+'/hand'
        self._SensorPosName = self._BaseName+'/pos'
        self._SensorOriName = self._BaseName+'/ori'
        self._SensorLinVelName = self._BaseName+'/v'
        self._SensorRotVelName = self._BaseName+'/w'
        self._SensorForceName = self._BaseName+'/force'
        self._SensorTorqueName = self._BaseName+'/torque'

        self.scene = haptix.mjInterface(dll_path=dll_path)
        self._connected = False
        if self.scene.mj_connected()==0:
            if self.scene.mj_connect(None)==0:
                self._connected = True
                print(f'Robot {robot_name} connected to Haptix')
            else:
                raise Exception('Connection to Haptix simulator failed')
        else:    
            self._connected = True
            print(f'Robot {robot_name} connected to Haptix')
        
        self._info = self.scene.mj_info()
        if self._info.nu>0:
            self._ctrl = self.scene.mj_get_control()

        self.tsamp = 0.01        # sampling rate 
        self.Init()    
    
    def __del__(self):
        if self.scene.mj_close()==0:
            self.Message('Haptix scene disconnected')   

    def Close(self):
        if self.scene.mj_close()==0:
            self.Message('Haptix scene disconnected')   

    def Init(self):
        self._JointHandles = [None]*self.nj
        self._ActuatorHandles = [None]*self.nj
        for i in range(self.nj):
            self._JointHandles[i]=self.scene.mj_name2id('joint',self._JointNames[i])
            self._ActuatorHandles[i]=self.scene.mj_name2id('actuator',self._ActuatorNames[i])
        
        self._BaseHandle = self.scene.mj_name2id('body',self._BaseName)
        if self._BaseHandle>=0:
            bb = self.scene.mj_get_body()
            body_pos=np.array(bb.pos[:bb.nbody])
            body_mat=np.array(bb.mat[:bb.nbody]).reshape((-1,3,3))
            self.TBase = map_pose(R=body_mat[self._BaseHandle], p=body_pos[self._BaseHandle], out='T')
        
        i1 = self.scene.mj_name2id('site',self._EEName)
        i2 = self.scene.mj_name2id('site',self._TCPName)
        if i1>=0 and i2>=0:
            si = self.scene.mj_get_site()
            site_pos=np.array(si.pos[:si.nsite])
            site_mat=np.array(si.mat[:si.nsite]).reshape((-1,3,3))
            pEE = site_pos[i1]
            REE = site_mat[i1]
            pHand = p=site_pos[i2]
            RHand = R=site_mat[i2]
            self.TCP = map_pose(R=REE@RHand.T,  p=REE@(pHand-pEE), out='T')
        
        idx = [None]*6
        idx[0] = self.scene.mj_name2id('sensor',self._SensorPosName)
        idx[1] = self.scene.mj_name2id('sensor',self._SensorOriName)
        idx[2] = self.scene.mj_name2id('sensor',self._SensorLinVelName)
        idx[3] = self.scene.mj_name2id('sensor',self._SensorRotVelName)
        idx[4] = self.scene.mj_name2id('sensor',self._SensorForceName)
        idx[5] = self.scene.mj_name2id('sensor',self._SensorTorqueName)

        if np.all(np.array(idx)>=0):
            hh = [None]*6
            for i in range(6):
                adr = self._info.sensor_adr[idx[i]]
                dim = self._info.sensor_dim[idx[i]]
                hh[i] = list(range(adr,adr+dim))
            self._SensorHandles = hh
        else:
            self._SensorHandles = None

        self.GetState()
        self._command.q = self._actual.q
        self._command.qdot = self._actual.qdot
        self._command.x = self._actual.x
        self._command.v = self._actual.v
        self.ResetTime()    

    # def simtime(self):
    #     self._state = self.scene.mj_get_state()
    #     return self._state.time

    def GetState(self):
        if (self.simtime()-self._last_update)>self.tsamp:
            self._state = self.scene.mj_get_state()
            self._actual.q = np.take(self._state.qpos,self._JointHandles)
            self._actual.qdot = np.take(self._state.qvel,self._JointHandles)
            if self._info.nsensor>0 and self._SensorHandles is not None:
                sensor = self.scene.mj_get_sensor()
                self._actual.x=checkx(np.take(sensor.sensordata,self._SensorHandles[0]+self._SensorHandles[1]))
                self._actual.v=np.take(sensor.sensordata,self._SensorHandles[2]+self._SensorHandles[3])
                self._actual.FT=np.take(sensor.sensordata,self._SensorHandles[4]+self._SensorHandles[5])
            else:
                x, J = self.kinmodel()
                self._actual.x = x
                self._actual.v = J@self._actual.qdot
                self._actual.FT = np.zeros(6)
                self._actual.trq = np.zeros(self.nj)
                self._actual.trqExt = np.zeros(self.nj)
            self._tt=self.simtime()
            self._last_update = self.simtime()

    def isReady(self):
        return self._connected

    def isActive(self):
        s1 = self.scene.mj_get_state()
        sleep(0.1)
        s2 = self.scene.mj_get_state()
        return s2.time>s1.time

    def Restart(self, qpos=None):
        if self.isReady:
            if isvector(q, dim=self.info.nq):
                self.scene.mj_reset()
                self._command.q = np.take(qpos,self._JointHandles) 
                self.SendRobot_q(self._command.q)
                self.GetState()
                self._state.qpos = qpos
                self._state.qvel = np.zeros(self._state.nv)
                self.scene.mj_set_state(self._state)
            self.GetState()

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
        self.SendRobot_q(q)
        self._command.q = q
        self._command.qdot = qdot
        self._command.trq = trq
        x, J = self.kinmodel(q)
        self._command.x = x
        self._command.v = J@qdot
        self.Update()
    
    def SetStrategy(self, strategy):
        pass

    def SendRobot_q(self, q):
        self._command.q = q
        if self.isReady:
            for i, x in zip(self._ActuatorHandles, q):
                self._ctrl.ctrl[i] = x
            self.scene.mj_set_control(self._ctrl) 

    def SendCtrl(self, u):
        self._command.q = np.take(q, self._ActuatorHandles)
        if self.isReady:
            self._ctrl.ctrl = u
            self.scene.mj_set_control(self._ctrl) 

    def GetAuxPos(self, idx):
        if self.isReay():
            self._state = self.scene.mj_get_state()
            return np.take(self._state,idx)

    def SendAuxCtrl(self, idx, val):
        if self.isReady() and self._info.nu>0:
            self._ctrl = self.scene.mj_get_control()
            for i, x in zip(idx, val):
                self._ctrl.ctrl[i] = x
            self.scene.mj_set_control(self._ctrl) 

    def SetMocapPose(self, ide, x):
        """Set pose of a mocap body

        Parameters
        ----------
        ide : str or int
            mocap body name or id
        x : array of floats
            mocap pose

        Raises
        ------
        ValueError
            Wrong pose shape

        Note
        ----
        When mocap body names are used, mocap bodies have to be first bodies in the model!
        """
        if self.isReady and self._info.nmocap>0:
            mocap = self.scene.mj_get_mocap()
            if isinstance(ide, str):
                idx = self.scene.mj_name2id('body', ide)-1 # world is body 0!
            else:
                idx = ide
            x = self.spatial(x)    
            if x.shape==(4, 4):
                xx = map_pose(T=x)
                for i in range(3):
                    mocap.pos[idx][i] = xx[i]
                for i in range(4):
                    mocap.quat[idx][i] = xx[i+3]
            elif x.shape==(3, 3):
                xx = r2q(x)
                for i in range(4):
                    mocap.quat[idx][i] = x[i]
            elif isvector(x, dim=7):
                for i in range(3):
                    mocap.pos[idx][i] = x[i]
                for i in range(4):
                    mocap.quat[idx][i] = x[i+3]
            elif isvector(x, dim=3):
                for i in range(3):
                    mocap.pos[idx][i] = x[i]
            elif isvector(x, dim=4):
                for i in range(4):
                    mocap.quat[idx][i] = x[i]
            else:
                raise ValueError(f'Parameter shape {x.shape} not supported')
            self.scene.mj_set_mocap(mocap)

    def GetObjectPose(self, typ, ide, out='x'):
        if self.isReady and (self._info.nmocap>0) and (typ in set(['body','site','geom'])):
            if isinstance(ide, str):
                idx = self.scene.mj_name2id(typ, ide)
            else:
                idx = ide
            val = eval('self.scene.mj_get_'+typ+'()')
            return map_pose(p=np.array(val.pos[idx]), R=np.array(val.mat[idx]).reshape(3,3),  out=out)

    def HaptixMessage(self, msg):
        self.scene.mj_message(msg)

    def sim(self, dt):
        if self.isReady():
            self._state = self.scene.mj_get_state()
            t0 = self._state.time
            t1 = t0
            while t1<t0+dt:
                sensor = self.scene.mj_update(self._ctrl)
                t1 = sensor.time
            self.GetState()
            self.Update()

class panda_haptix(robot_haptix, robot_no_compliance, panda):
    def __init__(self, robot_name='Panda', **kwargs):
        panda.__init__(self)
        kwargs.setdefault('dll_path',None)
        kwargs.setdefault('host',None)
        robot_haptix.__init__(self, robot_name, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_haptix.__del__(self)
        self.Message("Robot deleted",2)

class lwr_haptix(robot_haptix, robot_no_compliance, lwr):
    def __init__(self, robot_name='LWR', **kwargs):
        lwr.__init__(self)
        kwargs.setdefault('dll_path',None)
        kwargs.setdefault('host',None)
        robot_haptix.__init__(self, robot_name, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_haptix.__del__(self)
        self.Message("Robot deleted",2)

class ur10_haptix(robot_haptix, robot_no_compliance, ur10):
    def __init__(self, robot_name='UR10', **kwargs):
        ur10.__init__(self)
        kwargs.setdefault('dll_path',None)
        kwargs.setdefault('host',None)
        robot_haptix.__init__(self, robot_name, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_haptix.__del__(self)
        self.Message("Robot deleted",2)

class ur5_haptix(robot_haptix, robot_no_compliance, ur5):
    def __init__(self, robot_name='UR5', **kwargs):
        ur5.__init__(self)
        kwargs.setdefault('dll_path',None)
        kwargs.setdefault('host',None)
        robot_haptix.__init__(self, robot_name, **kwargs )
        self.__dict__.update(kwargs)
        self.Init()

    def __del__(self):
        robot_haptix.__del__(self)
        self.Message("Robot deleted",2)


if __name__ == '__main__':
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    r = panda_haptix()
    print('Haptix ? ', str(isHaptix(r.scene)))
    for i in range(r._info.nbody):
        print(f'Body {i} - {r.scene.mj_id2name("body",i)}')
    print('q: ',r.q)
    print('x: ',r.x)

    r.JMove(r.q_home, 2)
    r.SetMocapPose('Target',r.x)
    print('Body: ',r.GetObjectPose('body',10))


