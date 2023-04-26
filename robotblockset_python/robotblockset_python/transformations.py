import numpy as np
import quaternionic as Quaternion

from robotblockset_python.validateargs import *

_eps = np.finfo(np.float64).eps

def map_pose(x=None, T=None, Q=None, R=None, p=None, RPY=None, out='x'):
    """Convert pose form (SE3)

    A spatial pose can be represented as homogenous matrix or 7-dimensional
    array (translation and quaternion)

    Parameters
    ----------
    x : array of floats, optional
        Cartesian pose represented by translation and quaternion (7,)
    T : array of floats, optional
        Cartesian pose represented as homogenous matrix (4, 4)
    Q : array of floats, optional
        quaternion (4,)
    R : array of floats, optional
        rotation matrix (3, 3)
    p : array of floats, optional
        translation vector (3,)
    RPY : array of floats, optional
        rotation as Euler angles Roll, Pitch and Yaw (3,)
    out: str, optional
        output form (``T``: Homogenous matrix, ``X``: pose array,
        ``pR``: rotation matrix and translation))

    Returns
    -------
    ndarray
        quaternion (4,) or homogenous matrix (4, 4)

    Raises
    ------
    ValueError
        Not supported output form

    Note
    ----
    X takes precendece over T and boath over others. Q takes precedence over R
    """
    if x is not None:
        x = vector(x, dim=7)
        p = x[:3]
        Q = x[3:7]
    elif T is not None:
        T = matrix(T, shape=(4, 4))
        p = T[:3,3]
        Q = r2q(T[:3,:3])
    else:        
        if p is None:
            p = np.zeros(3)
        else:
            p = vector(p, dim=3)
        if Q is not None:
            Q = vector(Q, dim=4)
        elif R is not None:
            R = matrix(R, shape=(3,3))
            Q = r2q(R)
        elif RPY is not None:
            Q = Quaternion.array(rpy2r(RPY)).ndarray
            if Q[0]<0:
                Q = -Q
        else:
            Q = np.array([1, 0, 0, 0])
            R = np.eye(3)
    if out=='x':
        return np.hstack((p, Q))
    elif out=='T':
        if R is None:
            R = Quaternion.array(Q).to_rotation_matrix
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = p
        return T
    elif out=='pR':
        if R is None:
            R = Quaternion.array(Q).to_rotation_matrix
        return p, R
    elif out=='Q':
        return Q
    elif out=='R':
        if R is None:
            R = Quaternion.array(Q).to_rotation_matrix
        return R
    elif out=='p':
        return p
    else:
        raise ValueError(f'Output form {out} not supported')

def rp2t(R=None, p=None, Q=None,  RPY=None, out='T'):
    """Convert rotation and translation to pose (SE3)

    A spatial pose can be represented as homogenous matrix or 7-dimensional
    array (translation and quaternion)

    Parameters
    ----------
    Q : array of floats, optional
        quaternion (4,)
    R : array of floats, optional
        rotation matrix (3, 3)
    p : array of floats, optional
        translation vector (3,)
    RPY : array of floats, optional
        rotation as Euler angles Roll, Pitch and Yaw (3,)
    out: str, optional
        output form (``T``: Homogenous matrix, ``X``: pose array))

    Returns
    -------
    ndarray
        quaternion (4,) or homogenous matrix (4, 4)

    Raises
    ------
    ValueError
        Not supported output form

    Note
    ----
    Q takes precedence over R
    """
    return map_pose(Q=Q, R=R, p=p, RPY=RPY, out=out)

def checkx(x):
    """Make quaternion scalar component positive

    Parameters
    ----------
    x : array of floats
        spatial pose to check

    Returns
    -------
    array of floats
        pose with positive quaternion scalar component
    """
    x = np.asarray(x)
    if isvector(x, dim=7):
        if x[3]<0:
            x[3:] = -x[3:]
    else:
        Q = x[...,3:]
        Q[np.where(Q[...,0]<0)] = -Q[np.where(Q[...,0]<0)]
        x[...,3:] = Q
    return x

def checkQ(Q):
    """Make quaternion scalar component positive

    Parameters
    ----------
    Q : quaternion array
        quaternion to check

    Returns
    -------
    quaternion array
        quaternion with positive scalar component
    """
    Q = np.asarray(Q)
    if isvector(Q, dim=4):
        if Q[0]<0:
            Q = -Q
    else:
        Q[np.where(Q[...,0]<0)] = -Q[np.where(Q[...,0]<0)]
    return Q

def q2Q(Q):
    """Quaternion array to quaternion object

    Parameters
    ----------
    Q : array of floats
        quaternion (4,) or (...,4)

    Returns
    -------
    array of quaternions
        quaternion object
    """
    if check_shape(Q, shape=4):
        return Quaternion.array(Q)

def Q2q(Q):
    """Quaternion object to quaternion array

    Parameters
    ----------
    Q : quaternion
        quaternion object

    Returns
    -------
    array of floats
        quaternion array (4,) or (..., 4)
    """
    return Q.ndarray

def q2r(Q):
    """Quaternion to rotation matrix

    Parameters
    ----------
    Q : array of floats
        quaternion (4,) or (...,4)

    Returns
    -------
    array of floats
        rotation matrix (3, 3) or (..., 3, 3)
    """
    return Quaternion.array(Q).to_rotation_matrix

def r2q(R):
    """Rotation matrix to quaternion

    Parameters
    ----------
    R : array of floats
        rotation matrix (3, 3) or (...,3, 3)

    Returns
    -------
    array of floats
        quaternion (4, ) or (..., 4)
    """
    Q = Quaternion.array.from_rotation_matrix(R).ndarray
    if isvector(Q, dim=4):
        if Q[0]<0:
            Q = -Q
    else:
        Q[np.where(Q[...,0]<0)] = -Q[np.where(Q[...,0]<0)]
    return Q

def q2rpy(Q,out='rad'):
    R = q2r(Q)
    rpy = r2rpy(R, out=out)
    return rpy

def r2rpy(R, out = 'rad'):
    """ Rotation matrix to roll/pitch/yaw angles 
        Parameters
        ----------
              R       Rotation matrix (3x3)
        Returns
        ----------
              rpy     Euler angles(1x3) 
    """
    assert out in ['deg', 'rad']
    #rpy = np.zeros(3)
    #for i in range(0,3):
    #    rpy[i] = r2rpy_single(R[:,i])
    rpy = r2rpy_single(R, out=out)
    return rpy

def r2rpy_single(R, out = 'rad'):
    """ a """
    assert out in ['deg', 'rad'] 
    rpy = np.zeros(3)
    eps = np.finfo(np.float32).eps
    if (np.abs(R[0,0]) < eps) and (np.abs(R[1,0]) < eps):
        rpy[0] = 0
        rpy[1] = np.arctan2(-R[2,0], R[0,0])
        rpy[2] = np.arctan2(-R[1,2], R[1,1])

    else:
        rpy[0] = np.arctan2(R[1,0], R[0,0])
        sp = np.sin(rpy[0])
        cp = np.cos(rpy[0])
        rpy[1] = np.arctan2(-R[2,0], cp*R[0,0] + sp* R[1,0])
        rpy[2] = np.arctan2(sp* R[0,2] - cp * R[1,2], cp*R[1,1] - sp * R[0,1])
    if out == 'deg':
        rpy = 180*rpy/np.pi
    return rpy

def x2x(x):
    """Any pose to Cartesian pose

    Parameters
    ----------
    x : array of floats
        Pose (7,) or (4,4) or (3, 4)

    Returns
    -------
    array of floats
        Cartesian pose (7,)
    """
    x = np.asarray(x)
    if x.shape==(4, 4):
        return map_pose(T=x)
    elif x.shape==(3, 4):
        return map_pose(T=np.vstack((x,np.array(0,0,0,1))))
    elif isvector(x, dim=7):
        return x
    else:
        raise TypeError(f'Pose shape {x.shape} not supported')

def x2t(x):
    """Cartesian pose to homogenous matrix

    Parameters
    ----------
    x : array of floats
        Cartesian pose (7,) or (...,7)

    Returns
    -------
    array of floats
        homogenous matrix (4, 4) or (..., 4, 4)
    """
    x = np.asarray(x)
    if isvector(x, dim=7):
        T = rp2t(Q=x[3:7], p=x[:3], out='T')
    elif ismatrix (x, shape=7):
        T = np.array([rp2t(Q=xx[3:7], p=xx[:3], out='T') for xx in x])
    else:
        raise TypeError(f'Expected parameter shape (...,7) but is {x.shape}')
    return T

def t2x(x):
    """Homogenous matrix to cartesian pose

    Parameters
    ----------
    x : array of floats
        Cartesian pose (7,) or (...,7)

    Returns
    -------
    array of floats
        homogenous matrix (4, 4) or (..., 4, 4)
    """
    x = np.asarray(x)
    if check_shape(x, shape=(4,4)):
        p = x[...,:3,3]
        R = x[...,:3,:3]
        return np.hstack((p,r2q(R)))
    else:
        raise TypeError(f'Expected parameter shape (...,4,4) but is {x.shape}')

def rpy2r(rpy, *args, out='R', unit='rad'):
    """Euler angles RPY to rotation

    Parameters
    ----------
    rpy : float or array of floats
        Euler angles Roll or RPY
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)
    unit : str, optional
        angular unit (``rad`` or ``deg``)

    Args
    ----
    p : floar or array of floats, optional
        Euler angle pitch
    y : floar or array of floats, optional
        Euler angle yaw

    Returns
    -------
    array of floats
        quaternion or rotation matrix

    Raises
    ------
    ValueError
        Not supported output form
    """
    fac = getunit(unit=unit)
    if len(args)>0:
        y = np.asarray(rpy)*fac
        p = np.asarray(args[0])*fac
        r = np.asarray(args[1])*fac
    else:
        y = np.asarray(rpy[0])*fac
        p = np.asarray(rpy[1])*fac
        r = np.asarray(rpy[2])*fac
    Q = np.empty(np.broadcast(r, p, y).shape + (4,))
    Q[..., 0] = np.cos(r/2)*np.cos(p/2)*np.cos(y/2) + np.sin(r/2)*np.sin(p/2)*np.sin(y/2)
    Q[..., 1] = np.sin(r/2)*np.cos(p/2)*np.cos(y/2) - np.cos(r/2)*np.sin(p/2)*np.sin(y/2)
    Q[..., 2] = np.cos(r/2)*np.sin(p/2)*np.cos(y/2) + np.sin(r/2)*np.cos(p/2)*np.sin(y/2)
    Q[..., 3] = np.cos(r/2)*np.cos(p/2)*np.sin(y/2) - np.sin(r/2)*np.sin(p/2)*np.cos(y/2)

    if out=='Q':
        return Quaternion.array(Q).ndarray
    elif out=='R':
        return Quaternion.array(Q).to_rotation_matrix
    else:
        raise ValueError(f'Output form {out} not supported')

def rot_x(phi, out='R', unit='rad'):
    """Rotation matrix for rotation around x-axis

    Parameters
    ----------
    phi : int or float
        rotation angle
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)
    unit : str, optional
        angular unit (``rad`` or ``deg``)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    TypeError
        Parameters is not scalar
    """
    if isscalar(phi):
        phi = phi*getunit(unit=unit)
        cx = np.cos(phi)
        sx = np.sin(phi)
        R = np.array([
            [1,  0,   0],
            [0, cx, -sx],
            [0, sx,  cx]])
        if out=='R':
            return R
        elif out=='Q':
            return Quaternion.array.from_rotation_matrix(R).ndarray
        else:
            raise ValueError(f'Output form {out} not supported')
    else:
        raise TypeError('Parameter has to be scalar')

def rot_y(phi, out='R', unit='rad'):
    """Rotation matrix for rotation around y-axis

    Parameters
    ----------
    phi : int or float
        rotation angle
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)
    unit : str, optional
        angular unit (``rad`` or ``deg``)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    TypeError
        Parameters is not scalar
    """
    if isscalar(phi):
        phi = np.array(phi)*getunit(unit=unit)
        cx = np.cos(phi)
        sx = np.sin(phi)
        R = np.array([
            [cx,  0, sx],
            [0,   1,  0],
            [-sx, 0, cx]])
        if out=='R':
            return R
        elif out=='Q':
            return Quaternion.array.from_rotation_matrix(R).ndarray
        else:
            raise ValueError(f'Output form {out} not supported')
    else:
        raise TypeError('Parameter has to be scalar')

def rot_z(phi, out='R', unit='rad'):
    """Rotation matrix for rotation around z-axis

    Parameters
    ----------
    phi : int or float
        rotation angle
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)
    unit : str, optional
        angular unit (``rad`` or ``deg``)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    TypeError
        Incorect parameter type
    """
    if isscalar(phi):
        phi = np.array(phi)*getunit(unit=unit)
        cx = np.cos(phi)
        sx = np.sin(phi)
        R = np.array([
            [cx, -sx, 0],
            [sx,  cx, 0],
            [0,    0, 1]])
        if out=='R':
            return R
        elif out=='Q':
            return Quaternion.array.from_rotation_matrix(R).ndarray
        else:
            raise ValueError(f'Output form {out} not supported')
    else:
        raise TypeError('Parameter has to be scalar')

def rot_v(v, *phi, out='R', unit='rad'):
    """Rotation matrix for rotation around v-axis

    if phi is not defined, rotation angle equals norm of ``v``

    Parameters
    ----------
    v : array-like
        3-dimensional rotation axis
    *phi : int or float, optional
        rotation angle
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)
    unit : str, optional
        angular unit (``rad`` or ``deg``)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    TypeError
        Incorect parameter type
    """
    v = vector(v, dim=3)
    if out=='R':
        if not phi:
            phi = np.linalg.norm(v)
            v = v/phi
            unit='rad'
        else:
            phi = phi[0]
            v = v/np.linalg.norm(v)
        if isscalar(phi):
            phi = np.array(phi)*getunit(unit=unit)
            cx = np.cos(phi)
            sx = np.sin(phi)
            vx = 1-cx
            R = np.array([
                [cx,      -v[2]*sx,  v[1]*sx],
                [v[2]*sx,       cx, -v[0]*sx],
                [-v[1]*sx, v[0]*sx,       cx]])
            vv = v.reshape(3,1)
            R = (vv@vv.T)*vx+R
            return R
        else:
            raise TypeError('Parameter has to be scalar')
    elif out=='Q':
        if not phi:
            return Quaternion.array.from_rotation_vector(v).ndarray
        else:
            v = v/np.linalg.norm(v)*phi[0]
            return Quaternion.array.from_rotation_vector(v).ndarray
    else:
        raise ValueError(f'Output form {out} not supported')

def vx2r(v, out='R'):
    """Rotation matrix to rotate x-axis to vector

    Parameters
    ----------
    v : array-like
        3-dimensional vector
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    """
    v = vector(v, dim=3)
    v = v/np.linalg.norm(v)
    u = np.array([1, 0, 0])
    k = np.cross(u, v)
    if np.all(k<_eps):
        if v[1]<0:
            R = np.diag([-1, -1, 1])
        else:
            R = np.eye(3)
    else:
        costheta = np.dot(u,v)
        R = v2s(k)
        kk = k.reshape(3,1)
        R = costheta*np.eye(3)+R+(kk@kk.T)*(1-costheta)/np.linalg.norm(k)**2
    if out=='R':
        return R
    elif out=='Q':
        return Quaternion.array.from_rotation_matrix(R).ndarray
    else:
        raise ValueError(f'Output form {out} not supported')

def vy2r(v, out='R'):
    """Rotation matrix to rotate y-axis to vector

    Parameters
    ----------
    v : array-like
        3-dimensional vector
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    """
    v = vector(v, dim=3)
    v = v/np.linalg.norm(v)
    u = np.array([0, 1, 0])
    k = np.cross(u, v)
    if np.all(k<_eps):
        if v[2]<0:
            R = np.diag([1, -1, -1])
        else:
            R = np.eye(3)
    else:
        costheta = np.dot(u,v)
        R = v2s(k)
        kk = k.reshape(3,1)
        R = costheta*np.eye(3)+R+(kk@kk.T)*(1-costheta)/np.linalg.norm(k)**2
    if out=='R':
        return R
    elif out=='Q':
        return Quaternion.array.from_rotation_matrix(R).ndarray
    else:
        raise ValueError(f'Output form {out} not supported')

def vz2r(v, out='R'):
    """Rotation matrix to rotate z-axis to vector

    Parameters
    ----------
    v : array-like
        3-dimensional vector
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    """
    v = vector(v, dim=3)
    v = v/np.linalg.norm(v)
    u = np.array([0, 0, 1])
    k = np.cross(u, v)
    print(k)
    if np.all(np.abs(k)<_eps):
        if v[2]<0:
            print(v)
            R = np.diag([1, -1, -1])
        else:
            R = np.eye(3)
    else:
        costheta = np.dot(u,v)
        R = v2s(k)
        kk = k.reshape(3,1)
        R = costheta*np.eye(3)+R+(kk@kk.T)*(1-costheta)/np.linalg.norm(k)**2
    if out=='R':
        return R
    elif out=='Q':
        return Quaternion.array.from_rotation_matrix(R).ndarray
    else:
        raise ValueError(f'Output form {out} not supported')

def vv2r(u, v, out='R'):
    """Rotation matrix to rotate vector u to vector v

    Parameters
    ----------
    u, v : array-like
        3-dimensional vectors
    out : str, optional
        output form (``R``: rotation matrix, ``Q``: quaternion)

    Returns
    -------
    ndarray
        rotation matrix (3, 3) or quaternion (4,)

    Raises
    ------
    ValueError
        Not supported output form
    """
    v = vector(v, dim=3)
    v = v/np.linalg.norm(v)
    u = np.array([0, 0, 1])
    k = np.cross(u, v)
    if np.all(k<_eps):
        if v[2]<0:
            R = np.diag([1, -1, -1])
        else:
            R = np.eye(3)
    else:
        costheta = np.dot(u,v)
        R = v2s(k)
        kk = k.reshape(3,1)
        R = costheta*np.eye(3)+R+(kk@kk.T)*(1-costheta)/np.linalg.norm(k)**2
    if out=='R':
        return R
    elif out=='Q':
        return Quaternion.array.from_rotation_matrix(R).ndarray
    else:
        raise ValueError(f'Output form {out} not supported')

def ang4v(v1, v2, *vn,  unit='rad'):
    """Absolute angle between two vectors

    If vector ``vn`` is given, the angle is signed assuming ``vn`` is
    pointing in the same side as the normal.

    Parameters
    ----------
    v1, v2 : array-like
        3-dimensional vectors
    *vn : array-like, optional
        vector pointing in the direction of the normal
    unit : str, optional
        angular unit (``rad`` or ``deg``), by default ``rad``

    Returns
    -------
    ndarray
        angle between vectors
    """
    v1 = vector(v1, dim=3)
    v2 = vector(v2, dim=3)
    a = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
    if a>1:
        a = 1
    phi = np.arccos(a)
    if len(vn)>0:
        vn = vector(vn[0], dim=3)
        b = np.cross(v1,v2)
        if np.dot(np.array(vn),b)<0:
            phi = -phi
    return phi/getunit(unit=unit)

def side4v(v1, v2, vn):
    """Side of plane (v1,v2) vector vn is

    Parameters
    ----------
    v1, v2, vn : array-like
        3-dimensional vectors
    Returns
    -------
    int
        1: on same side as normal; -1: on opposite side; 0: on plane
    """
    v1 = vector(v1, dim=3)
    v2 = vector(v2, dim=3) 
    vn = vector(vn, dim=3)
    b = np.cross(v1,v2)
    return np.sign(np.dot(vn,b))

def v2s(v):
    """Map vector to matrix operator performing cross product

    Parameters
    ----------
    v : array-like
        3-dimensional vector

    Returns
    -------
    ndarray
        skew-symmertic matrix (3, 3)
    """
    v = vector(v, dim=3)
    S = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]])
    return S

def skew(v):
    """Map vector to matrix operator performing cross product

    Parameters
    ----------
    v : array-like
        3-dimensional vector

    Returns
    -------
    ndarray
        skew-symmertic matrix (3, 3)
    """
    return v2s(v)

def s2v(S):
    """Generate vector from skew-symmetric matrix

    Parameters
    ----------
    S : nparray
        (3, 3) skew-symmetric matrix

    Returns
    -------
    ndarray
        3-dimensional vector

    Raises
    ------
    TypeError
        Parameter shape error
    """
    if ismatrix(S, shape=(3, 3)) and isskewsymmetric(S):
        v = np.array([S[2,1]-S[1,2], S[0,2]-S[2,0], S[1,0]-S[0,1]])/2
        return v
    else:
        raise TypeError('Parameter has to be (3, 3) array')

def invskew(S):
    """Generate vector from skew-symmetric matrix

    Parameters
    ----------
    S : nparray
        (3, 3) skew-symmetric matrix

    Returns
    -------
    ndarray
        3-dimensional vector
    """
    return s2v(S)

def qlog(Q):
    """Log of quaternions

    Parameters
    ----------
    Q : quaternion array
        quaternion (4,) or (..., 4)

    Returns
    -------
    array of floats
        log og quaternion (4,) or (...,4)

    Raises
    ------
    TypeError
        Parameter shape error
    """
    Q = np.array(Q)
    if Q.shape[-1]==4:
        return np.log(Quaternion.array(Q)).ndarray
    else:
        raise TypeError('Parameter has to be (..., 4) array')

def qerr(Q2, *Q1):
    """Error of quaternions

    Angle between Q2 and Q1. If Q1 is ommited then Q2 is comapred
    to unit quaternion

    Parameters
    ----------
    Q2 : quaternion array
        quaternion (4,) or (..., 4)
    Q1 : quaternion array, optional
        quaternion (4,) or (..., 4)

    Returns
    -------
    array of floats
        distance between quaternions (3,) or (...,3)

    Raises
    ------
    TypeError
        Parameter shape error
    """
    if len(Q1)==0:
        return 2*qlog(Q2)[...,1:]
    else:
        Q1 = Q1[0]
        Q2 = np.array(Q2)
        Q1 = np.array(Q1)
        if Q2.shape==Q1.shape and Q2.shape[-1]==4:
            eq = 2*np.log(Quaternion.array(Q2)*Quaternion.array(Q1).inverse).ndarray
            return eq[...,1:]
        else:
            raise TypeError('Parameters have to be (..., 4) array')

def xerr(x2, x1):
    """Cartesian pose error

    Distance and angle betwee x2 and x1

    Parameters
    ----------
    x2 : array of floats
        Cartesian pose (7,) or (..., 7)
    x1 : array of floats
        Cartesian pose (7,) or (..., 7)

    Returns
    -------
    array of floats
        distance between Cartesian poses (7,) or (...,7)

    Raises
    ------
    TypeError
        Parameter shape error
    """
    x2 = np.asarray(x2)
    x1 = np.asarray(x1)
    if x2.shape==x1.shape and x2.shape[-1]==7:
        ep = x2[...,:3]-x1[...,:3]
        Q2 = x2[...,3:]
        Q1 = x1[...,3:]
        eq = qerr(Q2,Q1)
        return np.hstack((ep, eq))
    else:
        raise TypeError('Parameters have to be (..., 7) array')

def terr(T2, T1):
    """Homogenous matrix distance

    Distance between T2 and T1

    Parameters
    ----------
    T2 : array of floats
        Homogenous matrix (4, 4)
    T2 : array of floats
        Homogenous matrix (4, 4)

    Returns
    -------
    array of floats
        Homogenous matrix distance (4, 4)
    """
    T2 = matrix(T2, shape=(4, 4))
    T1 = matrix(T1, shape=(4, 4))
    Rerr = T2[:3,:3]@T1[:3,:3].transpose() # <- added transpose
    perr = T2[:3,3]-T1[:3,3]
    return rp2t(R=Rerr,p=perr, out='T')

if __name__ == '__main__':
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    print('Rot_x:\n',rot_x(45, out='R', unit='deg'))
    print('Q_x:\n',rot_x(45, out='Q', unit='deg'))
    print('RPY-Q:\n',rpy2r(45,0,0, out='Q', unit='deg'))
    print('RPY_R:\n',rpy2r(45,0,0, out='R', unit='deg'))
    print('Rot_v:\n',rot_v((1, 2, 3),1.2))
    print('vz2r:\n',vz2r((1, 0, 0)))
    print('ang4v:', ang4v((1,-2,3),(1,1,1),[2,2,3],unit='deg'))
    print('side4v:', side4v((1,-2,3),(1,1,1),[2,2,-3]))
    a=(1,2,3)
    S=skew(a)
    print('v:\n',a,'\nv2s(v)\n',S,'\ns2v(S)\n',s2v(S))

    Q0=rot_x(45, unit='deg')
    print('Q0: ',Q0)
    p0 = np.array([0, 1, 3])
    X0 = rp2t(Q=Q0, p=p0)
    print('T\n',rp2t(Q=Q0, p=p0, out='T'))

    p0 = np.array([0, 1, 3])
    p1 = np.array([1, 4, -1])
    p2 = np.array([-1, 1, 1])
    p3 = np.array([0, 3, 2])
    Q0=rot_x(0, unit='deg')
    Q1=rot_x(60, unit='deg')
    Q2=rot_y(30, unit='deg')
    Q3=rot_z(45, unit='deg')
    x0 = rp2t(Q=Q0, p=p0)
    x1 = rp2t(Q=Q1, p=p1)
    x2 = rp2t(Q=Q2, p=p2)
    x3 = rp2t(Q=Q3, p=p3)
    xx = np.vstack((x0,x1,x2,x3))

    print('Poses:\n',xx)
    print('T:\n',x2t(xx))

    Q5 = rot_y(0.27, unit='rad')
    QQ = np.vstack((Q0,Q1,Q2,Q3))
    ppa = np.vstack((p1,p0,p2,p3))
    QQa = np.vstack((Q5,Q1,-Q5,Q3))
    xxa = np.hstack((ppa,QQa))
    print('Err Q: ', qerr(QQa,QQ))
    print('Err x: ', xerr(xxa,xx))
    # p, Q = x2t(xx)
    # R = np.array([Quaternion.array(x).to_rotation_matrix for x in Q])
    # print(R)

    RRa = q2r(QQa)
    print('q2r:\n',RRa)
    print('r2q:\n',r2q(RRa))
    print('QQ\n',QQa)
    print('Check QQ\n', checkQ(QQa))

    print('xxa\n', xxa)
    print('Check xxa\n', checkx(xxa))
    
def t2p(T):
    return T[0:3,-1]

def t2r(T):
    return T[0:3, 0:3]
