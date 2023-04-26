"""
Interface to MuJoCo HAPTIX mjhaptix_user.dll (based on haptix.api) 
 
"""

# MIT License
#
# Copyright (c) 2020-2020 Leon Zlajpah
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# pylint: disable=invalid-name

from os import getcwd, path
from ctypes import *

hxMAXMOTOR = 32
hxMAXJOINT = 32
hxMAXCONTACTSENSOR = 32
hxMAXIMU = 32
mjMAXSZ = 1000

# API return codes

# Simple API ('hx')
class hxTime(Structure):
    """ creates a struct to match hxTime """
    _fields_ = [('sec', c_int),
                ('nsec', c_int)]

class hxRobotInfo(Structure):
    """ Description of the robot model being simulated """
    _fields_ = [('motor_count', c_int),
                ('joint_count', c_int),
                ('contact_sensor_count', c_int),
                ('imu_count', c_int),
                ('motor_limit', c_float*2*hxMAXJOINT),
                ('joint_limit', c_float*2*hxMAXJOINT),
                ('update_rate', c_float)]

class hxSensor(Structure):
    """ Sensor data returned from simulator """
    _fields_ = [('time_stamp', hxTime),
                ('motor_pos', c_float*hxMAXMOTOR),
                ('motor_vel', c_float*hxMAXMOTOR),
                ('motor_torque', c_float*hxMAXMOTOR),
                ('joint_pos', c_float*hxMAXJOINT),
                ('joint_vel', c_float*hxMAXJOINT),
                ('contact', c_float*hxMAXCONTACTSENSOR),
                ('imu_linear_acc', c_float*3*hxMAXIMU),
                ('imu_angular_vel', c_float*3*hxMAXIMU),
                ('imu_orientation', c_float*4*hxMAXIMU)]

class hxCommand(Structure):
    """ Motor commands sent to simulator """
    _fields_ = [('ref_pos', c_float*hxMAXMOTOR),
                ('ref_pos_enabled', c_int),
                ('ref_vel', c_float*hxMAXMOTOR),
                ('ref_vel_enabled', c_int),
                ('gain_pos', c_float*hxMAXMOTOR),
                ('gain_pos_enabled', c_int),
                ('gain_vel', c_float*hxMAXMOTOR),
                ('gain_vel_enabled', c_int)]

class hxInterface(object):
    """Creation of Haptix simple interface

    Parameters
    ----------
    path : str
        Path to the Haptix DLL mjhaptix_user.dll
    """
    def __init__(self, dll_path=None):
        if dll_path is None:
            dll_path = getcwd()
        dll_path = dll_path+'\\mjhaptix_user.dll'
        if path.isfile(dll_path):
            mjDll = cdll.LoadLibrary(dll_path)
        else:
            raise IOError(f'DLL {dll_path} does not exists')

        # Functions of hx Haptix
        self._internal_hx_connect = mjDll.hx_connect
        self._internal_hx_connect.argtypes = [c_char_p, c_int]
        self._internal_hx_connect.restype = c_int

        self._internal_hx_close = mjDll.hx_close
        self._internal_hx_close.argtypes = None
        self._internal_hx_close.restype = c_int

        self._internal_hx_robot_info = mjDll.hx_robot_info
        self._internal_hx_robot_info.argtypes = [POINTER(hxRobotInfo)]
        self._internal_hx_robot_info.restype = c_int

        self._internal_hx_update = mjDll.hx_update
        self._internal_hx_update.argtypes = [POINTER(hxCommand), POINTER(hxSensor)]
        self._internal_hx_update.restype = c_int

        self._internal_hx_read_sensors = mjDll.hx_read_sensors
        self._internal_hx_read_sensors.argtypes = [POINTER(hxSensor)]
        self._internal_hx_read_sensors.restype = c_int

        self._internal_hx_last_result = mjDll.hx_last_result
        self._internal_hx_last_result.argtypes = None
        self._internal_hx_last_result.restype = c_char_p

    # Functions of hx Haptix
    def hx_connect(self, ip, port):
        return self._internal_hx_connect(ip, int(port))

    def hx_close(self):
        self._internal_hx_close()

    def hx_robot_info(self, robot_info):
        self._internal_hx_robot_info(byref(robot_info))
        return robot_info

    def hx_update(self, command, sensor):
        self._internal_hx_update(byref(command), byref(sensor))
        return sensor

    def hx_read_sensors(self, sensor):
        self._internal_hx_read_sensors(byref(sensor))
        return sensor

    def hx_last_result(self):
        return self._internal_hx_last_result().decode('ASCII')


# Native API ('mj')
class mjState(Structure):
    """ State of the dynamical system in generalized coordinates """
    _fields_ = [('nq', c_int),
                ('nv', c_int),
                ('na', c_int),
                ('time', c_float),
                ('qpos', c_float*mjMAXSZ),
                ('qvel', c_float*mjMAXSZ),
                ('act', c_float*mjMAXSZ)]

class mjControl(Structure):
    """ Control signals """
    _fields_ = [('nu', c_int),
                ('time', c_float),
                ('ctrl', c_float*mjMAXSZ)]


class mjApplied(Structure):
    """ Applied forces """
    _fields_ = [('nv', c_int),
                ('nbody', c_int),
                ('time', c_float),
                ('qfrc', c_float*mjMAXSZ),
                ('xfrc', c_float*mjMAXSZ)]

class mjOneBody(Structure):
    """ Detailed information about one body """
    _fields_ = [('bodyid', c_int),
                ('isfloating', c_int),
                ('time', c_float),
                ('linacc', c_float*3),
                ('angacc', c_float*3),
                ('contactforce', c_float*3),
                ('pos', c_float*3),
                ('quat', c_float*4),
                ('linvel', c_float*3),
                ('angvel', c_float*3),
                ('force', c_float*3),
                ('torque', c_float*3)]

class mjMocap(Structure):
    """ Cartesian positions and orientations of mocap bodies (treated as constant by simulator """
    _fields_ = [('nmocap', c_int),
                ('time', c_float),
                ('pos', c_float*3*mjMAXSZ),
                ('quat', c_float*4*mjMAXSZ)]

class mjDynamics(Structure):
    """ Main output of forward dynamics; used internally to integrate the state """
    _fields_ = [('nv', c_int),
                ('na', c_int),
                ('time', c_float),
                ('qacc', c_float*mjMAXSZ),
                ('accdot', c_float*mjMAXSZ)]

class mjSensor(Structure):
    """ Sensor data; use the sensor desciptors in mjInfo to decode """
    _fields_ = [('nsensordata', c_int),
                ('time', c_float),
                ('sensordata', c_float*mjMAXSZ)]
                
class mjBody(Structure):
    """ Body positions and orientations in Cartesian coordinates (from forward kinematics) """
    _fields_ = [('nbody', c_int),
                ('time', c_float),
                ('pos', c_float*3*mjMAXSZ),
                ('mat', c_float*9*mjMAXSZ)]

class mjGeom(Structure):
    """ Geom positions and orientations in Cartesian coordinates """
    _fields_ = [('ngeom', c_int),
                ('time', c_float),
                ('pos', c_float*3*mjMAXSZ),
                ('mat', c_float*9*mjMAXSZ)]

class mjSite(Structure):
    """ Site positions and orientations in Cartesian coordinates """
    _fields_ = [('nsite', c_int),
                ('time', c_float),
                ('pos', c_float*3*mjMAXSZ),
                ('mat', c_float*9*mjMAXSZ)]

class mjTendon(Structure):
    """ Tendon lengths and velocities """
    _fields_ = [('ntendon', c_int),
                ('time', c_float),
                ('length', c_float*mjMAXSZ),
                ('velocity', c_float*mjMAXSZ)]

class mjActuator(Structure):
    """ Actuator lengths, velocities, and (scalar) forces in actuator space """
    _fields_ = [('nu', c_int),
                ('time', c_float),
                ('length', c_float*mjMAXSZ),
                ('velocity', c_float*mjMAXSZ),
                ('force', c_float*mjMAXSZ)]

class mjForce(Structure):
    """ Generalized forces acting on the system, resulting in dynamics:
        M(qpos)*qacc = nonconstraint + constraint """
    _fields_ = [('nv', c_int),
                ('time', c_float),
                ('nonconstraint', c_float*mjMAXSZ),
                ('constraint', c_float*mjMAXSZ)]

class mjContact(Structure):
    """ Information about all detected contacts """
    _fields_ = [('ncon', c_int),
                ('time', c_float),
                ('dist', c_float*mjMAXSZ),
                ('pos', c_float*3*mjMAXSZ),
                ('frame', c_float*9*mjMAXSZ),
                ('force', c_float*3*mjMAXSZ),
                ('geom1', c_float*mjMAXSZ),
                ('geom2', c_float*mjMAXSZ)]

class mjInfo(Structure):
    """ Static information about the model """
    _fields_ = [('nq', c_int),                     
                ('nv', c_int),                     
                ('na', c_int),                     
                ('njnt', c_int),                   
                ('nbody', c_int),                  
                ('ngeom', c_int),                  
                ('nsite', c_int),                  
                ('ntendon', c_int),                
                ('nu', c_int),                     
                ('neq', c_int),                    
                ('nkey', c_int),                   
                ('nmocap', c_int),                 
                ('nsensor', c_int),                
                ('nsensordata', c_int),            
                ('nmat', c_int),                   
                ('timestep', c_float),
                ('apirate', c_float),
                ('sensor_type', c_int*mjMAXSZ),
                ('sensor_datatype', c_int*mjMAXSZ),
                ('sensor_objtype', c_int*mjMAXSZ),
                ('sensor_objid', c_int*mjMAXSZ),
                ('sensor_dim', c_int*mjMAXSZ),
                ('sensor_adr', c_int*mjMAXSZ),
                ('sensor_noise', c_float*mjMAXSZ),
                ('jnt_type', c_int*mjMAXSZ),
                ('jnt_bodyid', c_int*mjMAXSZ),
                ('jnt_qposadr', c_int*mjMAXSZ),
                ('jnt_dofadr', c_int*mjMAXSZ),
                ('jnt_range', c_float*2*mjMAXSZ),
                ('geom_type', c_int*mjMAXSZ),
                ('geom_bodyid', c_int*mjMAXSZ),
                ('eq_type', c_int*mjMAXSZ),
                ('eq_obj1id', c_int*mjMAXSZ),
                ('eq_obj2id', c_int*mjMAXSZ),
                ('actuator_trntype', c_int*mjMAXSZ),
                ('actuator_trnid', c_int*2*mjMAXSZ),
                ('actuator_ctrlrange', c_float*2*mjMAXSZ)]

class mjInterface(object):
    """Creation of Haptix native interface

    Parameters
    ----------
    path : str
        Path to the Haptix DLL mjhaptix_user.dll
    """
    def __init__(self, dll_path=None):
        if dll_path is None:
            dll_path = getcwd()
        dll_path = dll_path+'\\mjhaptix_user.dll'
        if path.isfile(dll_path):
            mjDll = cdll.LoadLibrary(dll_path)
        else:
            raise IOError(f'DLL {dll_path} does not exists')


        # API get/set functions
        self._internal_mj_get_state = mjDll.mj_get_state
        self._internal_mj_get_state.argtypes = [POINTER(mjState)]
        self._internal_mj_get_state.restype = c_int

        self._internal_mj_get_control = mjDll.mj_get_control
        self._internal_mj_get_control.argtypes = [POINTER(mjControl)]
        self._internal_mj_get_control.restype = c_int

        self._internal_mj_get_applied = mjDll.mj_get_applied
        self._internal_mj_get_applied.argtypes = [POINTER(mjApplied)]
        self._internal_mj_get_applied.restype = c_int

        self._internal_mj_get_onebody = mjDll.mj_get_onebody
        self._internal_mj_get_onebody.argtypes = [POINTER(mjOneBody)]
        self._internal_mj_get_onebody.restype = c_int

        self._internal_mj_get_mocap = mjDll.mj_get_mocap
        self._internal_mj_get_mocap.argtypes = [POINTER(mjMocap)]
        self._internal_mj_get_mocap.restype = c_int

        self._internal_mj_get_dynamics = mjDll.mj_get_dynamics
        self._internal_mj_get_dynamics.argtypes = [POINTER(mjDynamics)]
        self._internal_mj_get_dynamics.restype = c_int

        self._internal_mj_get_sensor = mjDll.mj_get_sensor
        self._internal_mj_get_sensor.argtypes = [POINTER(mjSensor)]
        self._internal_mj_get_sensor.restype = c_int

        self._internal_mj_get_body = mjDll.mj_get_body
        self._internal_mj_get_body.argtypes = [POINTER(mjBody)]
        self._internal_mj_get_body.restype = c_int

        self._internal_mj_get_geom = mjDll.mj_get_geom
        self._internal_mj_get_geom.argtypes = [POINTER(mjGeom)]
        self._internal_mj_get_geom.restype = c_int

        self._internal_mj_get_site = mjDll.mj_get_site
        self._internal_mj_get_site.argtypes = [POINTER(mjSite)]
        self._internal_mj_get_site.restype = c_int

        self._internal_mj_get_tendon = mjDll.mj_get_tendon
        self._internal_mj_get_tendon.argtypes = [POINTER(mjTendon)]
        self._internal_mj_get_tendon.restype = c_int

        self._internal_mj_get_actuator = mjDll.mj_get_actuator
        self._internal_mj_get_actuator.argtypes = [POINTER(mjActuator)]
        self._internal_mj_get_actuator.restype = c_int

        self._internal_mj_get_force = mjDll.mj_get_force
        self._internal_mj_get_force.argtypes = [POINTER(mjForce)]
        self._internal_mj_get_force.restype = c_int
        
        self._internal_mj_get_contact = mjDll.mj_get_contact
        self._internal_mj_get_contact.argtypes = [POINTER(mjContact)]
        self._internal_mj_get_contact.restype = c_int

        self._internal_mj_set_state = mjDll.mj_set_state
        self._internal_mj_set_state.argtypes = [POINTER(mjState)]
        self._internal_mj_set_state.restype = c_int

        self._internal_mj_set_control = mjDll.mj_set_control
        self._internal_mj_set_control.argtypes = [POINTER(mjControl)]
        self._internal_mj_set_control.restype = c_int

        self._internal_mj_set_applied = mjDll.mj_set_applied
        self._internal_mj_set_applied.argtypes = [POINTER(mjApplied)]
        self._internal_mj_set_applied.restype = c_int

        self._internal_mj_set_onebody = mjDll.mj_set_onebody
        self._internal_mj_set_onebody.argtypes = [POINTER(mjOneBody)]
        self._internal_mj_set_onebody.restype = c_int

        self._internal_mj_set_mocap = mjDll.mj_set_mocap
        self._internal_mj_set_mocap.argtypes = [POINTER(mjMocap)]
        self._internal_mj_set_mocap.restype = c_int
        
        # Get and set rgba static data in simulator
        self._internal_mj_get_rgba = mjDll.mj_get_rgba
        self._internal_mj_get_rgba.argtypes = [c_char_p, c_int, POINTER(c_float)]
        self._internal_mj_get_rgba.restype = c_int

        self._internal_mj_set_rgba = mjDll.mj_set_rgba
        self._internal_mj_set_rgba.argtypes = [c_char_p, c_int, POINTER(c_float)]
        self._internal_mj_set_rgba.restype = c_int

        # API command and information functions
        self._internal_mj_connect = mjDll.mj_connect
        self._internal_mj_connect.argtypes = [c_char_p]
        self._internal_mj_connect.restype = c_int

        self._internal_mj_close = mjDll.mj_close
        self._internal_mj_close.argtypes = None
        self._internal_mj_close.restype = c_int

        self._internal_mj_result = mjDll.mj_result
        self._internal_mj_result.argtypes = None
        self._internal_mj_result.restype = c_int

        self._internal_mj_connected = mjDll.mj_connected
        self._internal_mj_connected.argtypes = None
        self._internal_mj_connected.restype = c_int

        self._internal_mj_info = mjDll.mj_info
        self._internal_mj_info.argtypes = [POINTER(mjInfo)]
        self._internal_mj_info.restype = c_int

        self._internal_mj_step = mjDll.mj_step
        self._internal_mj_step.argtypes = None
        self._internal_mj_step.restype = c_int
        
        self._internal_mj_update = mjDll.mj_update
        self._internal_mj_update.argtypes = [POINTER(mjControl), POINTER(mjSensor)]
        self._internal_mj_update.restype = c_int
        
        self._internal_mj_reset = mjDll.mj_reset
        self._internal_mj_reset.argtypes = [c_int]
        self._internal_mj_reset.restype = c_int
        
        self._internal_mj_equality = mjDll.mj_equality
        self._internal_mj_equality.argtypes = [c_int, c_int]
        self._internal_mj_equality.restype = c_int
        
        self._internal_mj_message = mjDll.mj_message
        self._internal_mj_message.argtypes = [c_char_p]
        self._internal_mj_message.restype = c_int
        
        self._internal_mj_name2id = mjDll.mj_name2id
        self._internal_mj_name2id.argtypes = [c_char_p, c_char_p]
        self._internal_mj_name2id.restype = c_int
        
        self._internal_mj_id2name = mjDll.mj_id2name
        self._internal_mj_id2name.argtypes = [c_char_p, c_int]
        self._internal_mj_id2name.restype = c_char_p

        self.return_codes = {
            0:   'OK',
            -1:  'BADSIZE',
            -2:  'BADINDEX',
            -3:  'BADTYPE',
            -4:  'BADCOMMAND',
            -5:  'NOMODEL',
            -6:  'CANNOTSEND',
            -7:  'CANNOTRECV',
            -8:  'TIMEOUT',
            -9:  'NOCONNECTION',
            -10: 'CONNECTED'}
            
        self.geom_type = ('PLANE', 'HFIELD', 'SPHERE', 'CAPSULE', 'ELLIPSOID', 'CYLINDER', 'BOX', 'MESH')
        
        self.sensor_type = ('TOUCH',                       # scalar contact normal forces summed over sensor zone
                            'ACCELEROMETER',               # 3D linear acceleration', in local frame
                            'VELOCIMETER',                 # 3D linear velocity', in local frame
                            'GYRO',                        # 3D angular velocity', in local frame
                            'FORCE',                       # 3D force between site's body and its parent body
                            'TORQUE',                      # 3D torque between site's body and its parent body
                            'MAGNETOMETER',                # 3D magnetometer
                            'RANGEFINDER',                 # distance geom along the positive size Z-axis
                            'JOINTPOS',                    # scalar joint position (hinge and slide only)
                            'JOINTVEL',                    # scalar joint velocity (hinge and slide only)
                            'TENDONPOS',                   # scalar tendon position
                            'TENDONVEL',                   # scalar tendon velocity
                            'ACTUATORPOS',                 # scalar actuator position
                            'ACTUATORVEL',                 # scalar actuator velocity
                            'ACTUATORFRC',                 # scalar actuator force
                            'BALLQUAT',                    # 4D ball joint quaterion
                            'BALLANGVEL',                  # 3D ball joint angular velocity
                            'FRAMEPOS',                    # 3D position
                            'FRAMEQUAT',                   # 4D unit quaternion orientation
                            'FRAMEXAXIS',                  # 3D unit vector: x-axis of object's frame
                            'FRAMEYAXIS',                  # 3D unit vector: y-axis of object's frame
                            'FRAMEZAXIS',                  # 3D unit vector: z-axis of object's frame
                            'FRAMELINVEL',                 # 3D linear velocity
                            'FRAMEANGVEL',                 # 3D angular velocity
                            'FRAMELINACC',                 # 3D linear acceleration
                            'FRAMEANGACC',                 # 3D angular acceleration
                            'SUBTREECOM',                  # 3D center of mass of subtree
                            'SUBTREELINVEL',               # 3D linear velocity of subtree
                            'SUBTREEANGMOM',               # 3D angular momentum of subtree
                            'USER') 
        self.joint_type = ('FREE', 'BALL', 'SLIDE', 'HINGE')
        
        self.transmission_type = ('JOINT', 'JOINTINPARENT', 'SLIDERCRANK', 'TENDON', 'SITE')
        
        self.constraint_type = ('CONNECT',                 # connect two bodies at a point (ball joint)
                                'WELD',                    # fix relative position and orientation of two bodies
                                'JOINT',                   # couple the values of two scalar joints with cubic
                                'TENDON',                  # couple the lengths of two tendons with cubic
                                'DISTANCE')                # fix the contact distance between two geoms        
  
    
    # Functions of mj Haptix: get/set functions
    def mj_get_state(self):
        """Read model state

        Returns:
        -------
        structure mjState
            nq : int  
                number of data in qpos
            nv : int
                number of data in qvel
            na : int
                number of data in act
            time : float)
                simulation time
            qpos : array [nq]
                generalized positions
            qvel : array [nv]
                generalized velocities
            act : array [na]
                actuator activations
        """        
        _state = mjState()
        self._internal_mj_get_state(byref(_state))
        return _state

    def mj_get_control(self):
        """Read control signals

        Returns:
        --------
        structure mjControl
            nu (int)        : number of data in ctrl
            time (float)    : simulation time
            ctrl[nu] (float): control array
        """        
        control = mjControl()
        self._internal_mj_get_control(byref(control))
        return control

    def mj_get_applied(self):
        """Read applied forces

        Returns:
        --------
        structure mjApplied
            nv (int)        : number of data in forces
            nbody (int)     : id of body
            time (float)    : simulation time
            qfrc[nv] (float): applied generalized forces 
            xfrc[nv] (float): Cartesian F/T applied to body
        """        
        applied = mjApplied()
        self._internal_mj_get_applied(byref(applied))
        return applied

    def mj_get_onebody(self, onebody):
        """Read information about one body

        Returns:
        --------
        structure mjOneBody
           bodyid; (int)    : body id, provided by user
           get only:            
           isfloating (int) : 1 if body is floating, 0 otherwise
           time (float)     : simulation time
           linacc[3] (float): linear acceleration
           angacc[3] (float): angular acceleration
           contactforce[3] (float) : net force from all contacts on this body
        get for all bodies; set for floating bodies only:
           pos[3] (float)   : position
           quat[4] (float)  : orientation quaternion
           linvel[3] (float): linear velocity
           angvel[3] (float): angular velocity
        get and set for all bodies: 
           force[3] (float) : Cartesian force applied to body CoM
           torque[3] (float): Cartesian torque applied to body
        """        
        self._internal_mj_get_onebody(byref(onebody))
        return onebody

    def mj_get_mocap(self):
        """Read mocaps

        Returns:
        --------
        structure mjMocap
            nmocap (int)           : number of mocap bodies
            time (float)           : simulation time
            pos[nmocap][3] (float) : positions
            quat[nmocap][4] (float): quaternion orientations
        """        
        mocap = mjMocap()
        self._internal_mj_get_mocap(byref(mocap))
        return mocap

    def mj_get_dynamics(self):
        """Read forward dynamics

        Returns:
        --------
        structure mjDynamics
            nv (int)          : number of generalized velocities
            na (int)          : number of actuator activations
            time (float)      : simulation time
            qacc[nv] (float)  : generalized accelerations
            actdot[na] (float): time-derivatives of actuator activations
        """        
        dynamics = mjDynamics()
        self._internal_mj_get_dynamics(byref(dynamics))
        return dynamics

    def mj_get_sensor(self):
        """Read sensor data; use the sensor desciptors in mjInfo to decode

        Returns:
        --------
        structure mjSensor
            nsensordata (int)              : number of data in arary
            time (float)                   : simulation time
            sensordata{nsensordata] (float): sensor data array
        """
        sensor = mjSensor()
        self._internal_mj_get_sensor(byref(sensor))
        return sensor

    def mj_get_body(self):
        """Read body positions

        Returns:
        --------
        structure mjBody
            nbody (int)          : number of bodies
            time (float)         : simulation time
            pos[nbody][3] (float): positions
            mat[nbody][9] (float): frame orientations
        """        
        body = mjBody()
        self._internal_mj_get_body(byref(body))
        return body

    def mj_get_geom(self):
        """Read geom positions

        Returns:
        --------
        structure mjGeom
            ngeom (int)          : number of geoms
            time (float)         : simulation time
            pos[ngeom][3] (float): positions
            mat[ngeom][9] (float): frame orientations
        """        
        geom = mjGeom()
        self._internal_mj_get_geom(byref(geom))
        return geom

    def mj_get_site(self):
        """Read site positions

        Returns:
        --------
        structure mjSite
            nsite (int)          : number of bodies
            time (float)         : simulation time
            pos[nsite][3] (float): positions
            mat[nsite][9] (float): frame orientations
        """        
        site = mjSite()
        self._internal_mj_get_site(byref(site))
        return site

    def mj_get_tendon(self):
        """Read tendons data

        Returns:
        --------
        structure mjTendon
            ntendon (int)            : number of tendons
            time (float)             : simulation time
            length[ntendon] (float)  : tendon lengths
            velocity[ntendon] (float): tendon velocities    
        """        
        tendon = mjTendon()
        self._internal_mj_get_tendon(byref(tendon))
        return tendon

    def mj_get_actuator(self):
        """Read tendons data

        Returns:
        --------
        structure mjTendon
            nu (int)            : number of actuators
            time (float)        : simulation time
            length[nu] (float)  : actuator lengths
            velocity[nu] (float): actuator velocities    
            force[nu] (float)   : actuator forces
        """   
        actuator = mjActuator()
        self._internal_mj_get_actuator(byref(actuator))
        return actuator

    def mj_get_force(self):
        """Read tendons data

        Returns:
        --------
        structure mjTendon
            nv (int)                 : number of generalized velocities/forces
            time (float)             : simulation time
            nonconstraint[nv] (float): sum of all non-constraint forces
            constraint[nv] (float)   : constraint forces (including contacts)
        """   
        force = mjForce()
        self._internal_mj_get_force(byref(force))
        return force

    def mj_get_contact(self):
        """Read tendons data

        Returns:
        --------
        structure mjTendon
            ncon (int)            : number of detected contacts
            time (float)          : simulation time
            dist[ncon             : contact normal distance
            pos[ncon][3] (float)  : contact position in world frame
            frame[ncon][9] (float): contact frame relative to world frame (0-2: normal)  
            force[ncon][3] (float): contact force in contact frame   
            geom1[ncon] (float)   : id of 1st contacting geom    
            geom2[ncon] (float)   : id of 2nd contacting geom (force: 1st -> 2nd)
        """   

        contact = mjContact()
        self._internal_mj_get_contact(byref(contact))
        return contact

    def mj_set_state(self, state):
        """Set simulator state

        Parameters:
        -----------
        state : structure mjState

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_set_state(byref(state))

    def mj_set_control(self, control):
        """Set control signals

        Parameters:
        -----------
        control : structure mjControl

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_set_control(byref(control))

    def mj_set_applied(self, applied):
        """Set applied forces

        Parameters:
        -----------
        applied : structure mjApplied

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_set_applied(byref(applied))

    def mj_set_onebody(self, onebody):
        """Set one body data

        Parameters:
        -----------
        onebody : structure mjOneBody

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_set_onebody(byref(onebody))

    def mj_set_mocap(self, mocap):
        """Set mocap positions

        Parameters:
        -----------
        mocap : structure mjMocap

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_set_mocap(byref(mocap))

    def mj_get_rgba(self, type, id):
        rgba = c_float(0.)
        self._internal_mj_get_rgba(bytes(type,'ASCII'), id, byref(rgba))
        return rgba.value

    def mj_set_rgba(self, type, id, rgba):
        rgba = c_float(rgba)
        return self._internal_mj_set_rgba(bytes(type,'ASCII'), id, byref(rgba))

    # Functions of mj Haptix: command and information functions
    def mj_connect(self, ip):
        """Connect to Haptix simulator

        Parameters:
        -----------
        ip : string
            host IP

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_connect(ip)

    def mj_close(self):
        """Close connection to Haptix simulator

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_close()

    def mj_result(self):
        """Returns last result code

        Returns:
        --------
            string: last recult code
        """
        return self._internal_mj_result()
        
    def mj_connected(self):
        """Returns connection status

        Returns:
        --------
            int: return 1 if connected to simulator, 0 otherwise
        """
        return self._internal_mj_connected()

    def mj_info(self):
        """Get static properties of current model

        Returns:
        --------
        myInfo: structure
                nq (int)                        : number of generalized positions
                nv (int)                        : number of generalized velocities
                na (int)                        : number of actuator activations
                njnt (int)                      : number of joints
                nbody (int)                     : number of bodies
                ngeom (int)                     : number of geoms
                nsite (int)                     : number of sites
                ntendon (int)                   : number of tendons
                nu (int)                        : number of actuators/controls
                neq (int)                       : number of equality constraints
                nkey (int)                      : number of keyframes
                nmocap (int)                    : number of mocap bodies
                nsensor (int)                   : number of sensors
                nsensordata (int)               : number of elements in sensor data array
                nmat (int)                      : number of materials
                timestep (float)                : simulation timestep
                apirate (float)                 : API update rate 
                sensor_type[nsensor] (int)      : sensor type 
                sensor_datatype[nsensor] (int)  : type of sensorized object
                sensor_objtype[nsensor] (int)   : type of sensorized object
                sensor_objid[nsensor] (int)     : id of sensorized object
                sensor_dim[nsensor] (int)       : number of sensor outputs
                sensor_adr[nsensor] (int)       : address in sensor data array
                sensor_noise[nsensor] (float)   : noise standard deviation
                jnt_type[njnt] (int)            : joint type (mjtJoint)
                jnt_bodyid[njnt] (int)          : id of body to which joint belongs
                jnt_qposadr[njnt] (int)         : address of joint position data in qpos
                jnt_dofadr[njnt] (int)          : address of joint velocity data in qvel
                jnt_range[njnt][2] (float)      : joint range  (0,0): no limits
                geom_type[ngeom] (int)          : geom type (mjtGeom)
                geom_bodyid[ngeom] (int)        : id of body to which geom is attached
                eq_type[neq] (int)              : equality constraint type (mjtEq)
                eq_obj1id[neq] (int)            : id of constrained object
                eq_obj2id[neq] (int)            : id of 2nd constrained object  -1 if not applicable
                actuator_trntype[nu] (int)      : transmission type (mjtTrn)
                actuator_trnid[nu][2] (int)     : transmission target id
                actuator_ctrlrange[nu][2](float): actuator control range (0,0): no limits
        """
        info = mjInfo()
        self._internal_mj_info(byref(info))
        return info

    def mj_step(self):
        """Advance simulation if paused, no effect if running

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_step()

    def mj_update(self, control):
        """Set control, step if paused or wait for 1/apirate if running, get sensor data

        Parameters:
        -----------
        control : structure mjControl

        Returns:
        --------
        int
            API return code
        """
        sensor = mjSensor()
        self._internal_mj_update(byref(control), byref(sensor))
        return sensor

    def mj_reset(self, keyframe):
        """Reset simulation to specified key frame

        Parameters:
        -----------
        keyframe : int
            key frame; -1: reset to model reference configuration

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_reset(keyframe)

    def mj_equality(self, eqid, state):
        """Modify state of specified equality constraint

        Parameters:
        -----------
        eqid : int
            equality id
        state : int
            equality constraint, 1: enable, 0: disable

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_equality(eqid, state)

    def mj_message(self, message):
        """Show text message in simulator

        Parameters:
        -----------
        message : string
            message, None: clear currently shown message

        Returns:
        --------
        int
            API return code
        """
        return self._internal_mj_message(bytes(message,'ASCII'))
        
    def mj_name2id(self, type, name):
        """Returns id of object with specified type and name
        
        valid object types: body, geom, site, joint, tendon, sensor, actuator, equality

        Parameters:
        -----------
        type : string)
            object type
        name : type
            object name

        Returns:
        --------
            int: object id (-1: not found; -2: error)
        """
        return self._internal_mj_name2id(bytes(type,'ASCII'), bytes(name,'ASCII'))
        
    def mj_id2name(self, type, id):
        """Returns name of object with specified type and id

        valid object types: body, geom, site, joint, tendon, sensor, actuator, equality

        Parameters:
        -----------
        type : string
            object type
        id : int
            object id

        Returns:
        --------
        string
            object name
        """
        return self._internal_mj_id2name(bytes(type,'ASCII'), id).decode('ASCII')
      

def isHaptix(scn):
    return isinstance(scn, mjInterface) or isinstance(scn, hxInterface)
