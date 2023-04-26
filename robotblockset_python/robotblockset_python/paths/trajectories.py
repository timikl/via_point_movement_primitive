import numpy as np
import quaternionic as Quaternion
from scipy.interpolate import interp1d

from validateargs import vector, ismatrix, check_option
from transformations import ang4v, rot_v

_eps = 100*np.finfo(np.float64).eps

def circ(p0, p1, pC, s):
    """Points on arc defined by two points and center point

    Parameters
    ----------
    p0 : array of floats
        inital arc point (3,)
    p1 : array of floats
        final arc point (3,)
    pC : array of floats
        arc center point (3,)
    s : float
        normalized arc distance

    Raises
    ------
    ValueError
        Points are not distinct
    """
    p0 = vector(p0, dim=3)
    p1 = vector(p1, dim=3)
    pC = vector(pC, dim=3)
    v1 = p0-pC
    v2 = p1-pC
    v1n = np.linalg.norm(v1)
    v2n = np.linalg.norm(v2)
    phi = ang4v(v1, v2)
    if v1n>0 and v2n>0 and phi>0:
        v3 = np.cross(v1, v2)
        return (1+s*(v2n-v1n)/v1n)*rot_v(v3, s*phi, out='R')@v1+pC
    else:
        raise ValueError('Points must be distinct')
    

def jline(q0, q1, t):
    """Trajectory form q0 to q1 with constant velocity

    Parameters
    ----------
    q0 : array of floats
        initial joint positions (n,)
    q1 : array of floats
        final joint position (n,)
    t : array of floats
        trajectory time (nsamp,)

    Returns
    -------
    array of floats
        interpolated joint position (nsamp, n)
    array of floats
        interpolated joint velocities  (nsamp, n)
    array of floats
        interpolated joint accelerations  (nsamp, n)
    """
    q0 = vector(q0)
    q1 = vector(q1)
    if q0.size==q1.size:
        t = t-t[0]
        if q0.size==1:
            qt = q0+(q1-q0)*t
        else:
            qt = q0+np.einsum('i,j->ji',q1-q0,t/t[-1])
        dq = (q1-q0)/t[-1]
        qdt = np.ones(qt.shape)*dq
        return qt, qdt, np.zeros(qt.shape)
    else:
        TypeError('Input vectors must be same size')

def jtrap(q0, q1, t, ta=0.1):
    """Trajectory form q0 to q1 with trapezoidal velocity

    Parameters
    ----------
    q0 : array of floats
        initial joint positions (n,)
    q1 : array of floats
        final joint position (n,)
    t : array of floats
        trajectory time (nsamp,)
    ta : float, optional
        acceleration/deceleration time

    Returns
    -------
    array of floats
        interpolated joint position (nsamp, n)
    array of floats
        interpolated joint velocities  (nsamp, n)
    array of floats
        interpolated joint accelerations  (nsamp, n)
    """
    q0 = vector(q0)
    q1 = vector(q1)
    if q0.size==q1.size:
        t = t-t[0]
        tm = np.max(t)
        acc = 1/(ta*(tm-ta))
        s = lambda t: (t<=ta)*0.5*acc*t**2+(t>ta and t<=(tm-ta))*(0.5*acc*ta**2+acc*ta*(t-ta))+(t>(tm-ta))*(1-0.5*acc*(tm-t)**2)
        v = lambda t: (t<=ta)*acc*t+(t>ta and t<=(tm-ta))*acc*ta+(t>(tm-ta))*acc*(tm-t)
        a = lambda t: (t<=ta)*acc-(t>(tm-ta))*acc
        st = np.array([s(x) for x in t])
        vt = np.array([v(x) for x in t])
        at = np.array([a(x) for x in t])
        if q0.size==1:
            qt = q0+(q1-q0)*st
            qdt = q0+(q1-q0)*vt
            qddt = q0+(q1-q0)*at
        else:
            qt = q0+np.einsum('i,j->ji',q1-q0,st)
            qdt = np.einsum('i,j->ji',q1-q0,vt)
            qddt = np.einsum('i,j->ji',q1-q0,at)
        return qt, qdt, qddt
    else:
        TypeError('Input vectors must be same size')

def jpoly(q0, q1, t, qd0=None, qd1=None):
    """Trajectory from q0 to q1 using 5th order polynomial

    Parameters
    ----------
    q0 : array of floats
        initial joint positions (n,)
    q1 : array of floats
        final joint position (n,)
    t : array of floats
        trajectory time (nsamp,)
    qd0 : array of floats
        Initial joint velocities (n,)
    qd1 : array of floats
        Final joint velocities (n,)

    Returns
    -------
    array of floats
        interpolated joint position (nsamp, n)
    array of floats
        interpolated joint velocities  (nsamp, n)
    array of floats
        interpolated joint accelerations  (nsamp, n)
    """
    q0 = vector(q0)
    q1 = vector(q1)
    if qd0 is None:
        qd0 = np.zeros(q0.shape)
    else:
        qd0 = vector(qd0)
    if qd1 is None:
        qd1 = np.zeros(q0.shape)
    else:
        qd1 = vector(qd1)
    if q0.size==q1.size and qd0.size==q0.size and qd1.size==q1.size:
        tmax = max(t)
        t = np.copy(vector(t).T)/tmax

        A =   6*(q1-q0)-3*(qd1+qd0)*tmax
        B = -15*(q1-q0)+(8*qd0+7*qd1)*tmax
        C =  10*(q1-q0)-(6*qd0+4*qd1)*tmax
        E = qd0*tmax
        F = q0
  
        tt = np.array([t**5, t**4, t**3, t**2, t, np.ones(t.shape)])
        s = np.array([A, B, C, np.zeros(A.shape), E, F]).reshape((6,q0.size))
        v = np.array([np.zeros(A.shape), 5 * A, 4 * B, 3 * C, np.zeros(A.shape), E]).reshape((6,q0.size))/tmax
        a = np.array([np.zeros(A.shape), np.zeros(A.shape), 20 * A, 12 * B, 6 * C, np.zeros(A.shape)]).reshape((6,q0.size))/tmax**2
        qt = np.einsum('ij,ik->kj',s,tt)
        qdt = np.einsum('ij,ik->kj',v,tt)
        qddt = np.einsum('ij,ik->kj',a,tt)
        if q0.size==1:
            return qt.flatten(), qdt.flatten(), qddt.flatten()
        else:
            return qt, qdt, qddt
    else:
        TypeError('Input vectors must be same size')

def jtraj(q0, q1, t, traj='poly', qd0=None, qd1=None):
    """Trajectory form q0 to q1 

    Parameters
    ----------
    q0 : array of floats
        initial joint positions (n,)
    q1 : array of floats
        final joint position (n,)
    t : array of floats
        trajectory time (nsamp,)
    typ : str
        trajectory type (`poly`, `trap` or `line`)
    qd0 : array of floats
        Initial joint velocities (n,)
    qd1 : array of floats
        Final joint velocities (n,)

    Returns
    -------
    array of floats
        interpolated joint position (nsamp, n)
    array of floats
        interpolated joint velocities  (nsamp, n)
    array of floats
        interpolated joint accelerations  (nsamp, n)
    """
    
    # Old. Vibration occurs, new code above to test whether this code below might be problematic.
    q0 = vector(q0)
    q1 = vector(q1)
    if check_option(traj, 'poly'):
        _traj = jpoly
    elif check_option(traj, 'trap'):
        _traj = jtrap
    elif check_option(traj, 'line'):
        _traj = jline
    else:
        raise ValueError(f'Trajectory type {traj} not supported')
    return _traj(q0, q1, t)

def cline(x0, x1, t, short=True):
    """Cartesian trajectory form x0 to x1 with constant velocity

    Initial and final pose are defined by position and quaternion.

    Parameters
    ----------
    x0 : array of floats
        initial Cartesian pose (7,)
    x1 : array of floats
        final Cartesian pose (7,)
    t : array of floats
        trajectory time  (nsamp,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    """
    x0 = vector(x0, dim=7)
    x1 = vector(x1, dim=7)
    s, _, _ = jline(0., 1., t)
    xt = xinterp(x0, x1, s, short=short)
    xdt = gradientCartesianPath(xt,t)
    xddt = gradientPath(xdt,t)
    return xt, xdt, xddt

def ctrap(x0, x1, t, short=True):
    """Cartesian trajectory form x0 to x1 with trapezoidal velocity

    Initial and final pose are defined by position and quaternion.

    Parameters
    ----------
    x0 : array of floats
        initial Cartesian pose (7,)
    x1 : array of floats
        final Cartesian pose (7,)
    t : array of floats
        trajectory time  (nsamp,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    """
    x0 = vector(x0, dim=7)
    x1 = vector(x1, dim=7)
    s, _, _ = jtrap(0., 1., t)
    xt = xinterp(x0, x1, s, short=short)
    xdt = gradientCartesianPath(xt,t)
    xddt = gradientPath(xdt,t)
    return xt, xdt, xddt

def cpoly(x0, x1, t, short=True):
    """Cartesian trajectory form x0 to x1 using 5th order polynomial

    Initial and final pose are defined by position and quaternion.

    Parameters
    ----------
    x0 : array of floats
        initial Cartesian pose (7,)
    x1 : array of floats
        final Cartesian pose (7,)
    t : array of floats
        trajectory time  (nsamp,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    """
    x0 = vector(x0, dim=7)
    x1 = vector(x1, dim=7)
    s, _, _ = jpoly(0., 1., t)
    xt = xinterp(x0, x1, s, short=short)
    xdt = gradientCartesianPath(xt,t)
    xddt = gradientPath(xdt,t)
    return xt, xdt, xddt

def ctraj(x0, x1, t, traj='poly', short=True):
    """Cartesian trajectory form x0 to x1 

    Initial and final pose are defined by position and quaternion.

    Parameters
    ----------
    x0 : array of floats
        initial Cartesian pose (7,)
    x1 : array of floats
        final Cartesian pose (7,)
    t : array of floats
        trajectory time  (nsamp,)
    traj : str
        trajectory type (`poly`, `trap` or `line`)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    """
    x0 = vector(x0, dim=7)
    x1 = vector(x1, dim=7)
    if check_option(traj, 'poly'):
        _traj = cpoly
    elif check_option(traj, 'trap'):
        _traj = ctrap
    elif check_option(traj, 'line'):
        _traj = cline
    else:
        raise ValueError(f'Trajectory type {traj} not supported')
    return _traj(x0, x1, t)

def carctraj(x0, x1, pC, t, traj='poly', short=True):
    """Cartesian trajectory on arc form x0 to x1

    Arc is defined by initial and final pose and arc center position.

    Parameters
    ----------
    x0 : array of floats
        initial Cartesian pose (7,)
    x1 : array of floats
        final Cartesian pose (7,)
    pC : array of floats
        arc center position (3,)
    t : array of floats
        trajectory time  (nsamp,)
    traj : str
        trajectory type (`poly`, `trap` or `line`)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    array of floats
        interpolated Cartesian pose (nsamp,7)
    """
    x0 = vector(x0, dim=7)
    x1 = vector(x1, dim=7)
    pc = vector(pC, dim=3)
    s, _, _ = jtraj(0., 1., t, traj=traj)
    xt = xarcinterp(x0, x1, pC, s, short=short)
    xdt = gradientCartesianPath(xt,t)
    xddt = gradientPath(xdt,t)
    return xt, xdt, xddt

def interp(y1,y2,s):
    """Multidimensional linear interpolation

    Returns linear interpolated data points between y1 and y2 at s

    Parameters
    ----------
    y1 : array of floats
        initial data points (n,)
    y2 : array of floats
        final data points (n,)
    s : array of floats
        query data points (ns,)

    Returns
    -------
    array of floats
        interpolated data points (ns, n)
    """
    y1 = vector(y1)
    y2 = vector(y2)
    s = vector(s)
    if y1.size==y2.size:
        if y1.size==1:
            return y1+(y2-y1)*s
        else:
            return y1+np.einsum('i,j->ji',y2-y1,s)
    else:
        TypeError('Input vectors must be same size')

def slerp(Q1, Q2, s, short=True):
    """Spherical linear interpolation of unit quaternion like arrays

    Returns linear interpolated data points between Q1 and Q2
    using SLERP

    Parameters
    ----------
    Q1 : array of floats
        initial quaternion (4,)
    Q2 : array of floats
        final quaternion (4,)
    s : array of floats
        query data points (ns,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    ndarray of quaternions
        interpolated quaternion elements (ns, 4)
    """
    Q1 = vector(Q1, dim=4)
    Q1 = Q1 / np.linalg.norm(Q1)

    Q2 = vector(Q2, dim=4)
    Q2 = Q2 / np.linalg.norm(Q2)


    s = np.asarray(s).flatten()

    qq = np.clip(np.dot(Q1, Q2),-1.,1.)
    if short:
        if qq<0:
            Q2 = -Q2  # pylint: disable=invalid-unary-operand-type
            qq = -qq  # pylint: disable=invalid-unary-operand-type
    else:
        if qq>0:
            Q2 = -Q2  # pylint: disable=invalid-unary-operand-type
            qq = -qq  # pylint: disable=invalid-unary-operand-type
    phi = np.arccos(qq)
    sinphi = np.sin(phi)
    n = s.size
    Q = np.empty((n, 4))
    for i in range(n):
        ss = s[i]
        if ss==0:
            Q[i] = Q1
        elif ss==1:
            Q[i] = Q2
        else:
            if abs(phi)<_eps:
                Q[i] = Q1
            else:
                Q[i] = (np.sin((1-ss)*phi)*Q1+np.sin(ss*phi)*Q2)/sinphi
    return Q

def qinterp(Q1, Q2, s, short=True):
    """Spherical linear interpolation of unit quaternion like arrays

    Returns linear interpolated data points between Q1 and Q2
    using SLERP

    Parameters
    ----------
    Q1 : array of floats
        initial quaternion (4,)
    Q2 : array of floats
        final quaternion (4,)
    s : array of floats
        query data points (ns,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    ndarray of quaternions
        interpolated quaternion elements (ns, 4)
    """
    return slerp(Q1, Q2, s, short=short)

def xinterp(x1, x2, s, short=True):
    """Linear interpolation of spatial poses (SE3)

    Returns linear interpolated data points between x1 and x2
    using LERP for positions and SLERP for rotations

    Spatial poses are represented as array of 3 positions and
    4 quaternion elements

    Parameters
    ----------
    x1 : array of floats
        initial Cartesian pose (7,)
    x2 : array of floats
        final Cartesian pose (7,)
    s : array of floats
        query data points (ns,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    ndarray
        interpolated Cartesian poses (ns, 7)
    """
    x1 = vector(x1, dim=7)
    x2 = vector(x2, dim=7)
    s = vector(s)
    p = interp(x1[:3], x2[:3], s)
    Q = qinterp(x1[3:], x2[3:], s, short=short)
    return np.hstack((p, Q))

def xarcinterp(x1, x2, pC, s, short=True):
    """Linear interpolation of spatial poses (SE3) along arc

    Returns linear interpolated data points between x1 and x2
    using LERP for positions on arc and SLERP for rotations

    Spatial poses are represented as array of 3 positions and
    4 quaternion elements

    Parameters
    ----------
    x1 : array of floats
        initial Cartesian pose (7,)
    x2 : array of floats
        final Cartesian pose (7,)
    pC : array of floats
        arc center position (3,)
    s : array of floats
        query data points (ns,)
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    ndarray
        interpolated Cartesian poses (ns, 7)
    """
    x1 = vector(x1, dim=7)
    x2 = vector(x2, dim=7)
    pC = vector(pC, dim=3)
    s = vector(s)
    p = [circ(x1[:3], x2[:3], pC, ss) for ss in s]
    Q = qinterp(x1[3:], x2[3:], s, short=short)
    return np.hstack((p, Q))

def interp1(s, y, si, kind = 'linear'):
    """Wrapper for SciPy interp1d

    Parameters
    ----------
    s : array of floats
        query points (ns,)
    y : array of floats
        data points (ns, n)
    si : array of floats
        quary points (ni,)

    Returns
    -------
    array of floats
        data in query points (ni, n)
    """
    #f = interp1d(s, y, axis=0)
    f = interp1d(s, y, axis=0, kind=kind, fill_value = 'extrapolate',bounds_error = False)
    return f(si)

def interpPath(s, path, squery):
    """ Interpolate path for query path values

    Parameters
    ----------
    s : array of floats
        path parameter (ns,)
    path : array of floats
        path data (ns, n)
    squery : array of floats
        query path points (ni, )

    Returns
    -------
    array of floats
        path values at query points (ni, n)
    """
    s = vector(s)
    path = np.array(path)
    if ismatrix(path) and path.shape[0]==len(s):
        return interp1(s, path, squery)
    else:
        raise TypeError(f's and path must have same first dimension, but have shapes {s.shape} and {path.shape}')
    
def uniqueQuaternion(Q):
    """
    %UNIQUEQUATERNIONPATH Correct the quaternions so that they are unique
    %
    % Usage:
    %       [q]=uniqueCartesianPath(q)
    %
    % Input:
    %       q   series of quaternions (n x 4) or cell array (n x quaternion)
    %
    % Output:
    %       q   corrected series of quaternions 
    %
    %

    % Copyright (c) 2019 by IJS Leon Zlajpah
    
    """
    0
    for i in range(1,Q.shape[0]):
        C = np.dot(Q[i-1, :], Q[i,:])
        if C<0:
            Q[i,:] = -Q[i,:]
            
    return Q

def interpQuaternionPath(s, path, squery, short=True):
    """ Interpolate quaternion path for query path values

    Returns sequentially linear interpolated data points between path points
    using SLERP.

    Parameters
    ----------
    s : array of floats
        path parameter (ns,)
    path : array of quaternions
        path quaternions (ns, 4)
    squery : array of floats
        query path points (ni, )
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of quaternions
        path quaternions at query points (ni, 4)
    """
    s = vector(s)
    path = np.array(path)
    if not ismatrix(path, shape=(len(s), 4)):
        raise TypeError(f'path must have dimension {(len(s), 4)}, but has {path.shape}')
    n = len(s)
    m = len(squery)
    try:
        i1 = np.clip(np.floor(interp1(s,range(n),squery, kind = 'cubic')),0,n-2).astype(int)
    except:
        i1 = np.clip(np.floor(interp1(s,range(n),squery, kind = 'quadratic')),0,n-2).astype(int)
    i2 = np.clip(i1+1,0,n-1).astype(int)
    ss = (squery-s[i1])/(s[i2]-s[i1])
    newpath = np.empty(shape=(m,4))
    for i in range(m):
        xx = qinterp(path[i1[i],:], path[i2[i],:], ss[i], short=short)
        newpath[i,:] = xx
    return newpath

def interpCartesianPath(s, path, squery, short=True):
    """ Interpolate Cartesian path for query path values

    Returns sequentially linear interpolated data points between path points
    using LERP for positions and SLERP for rotations.

    Spatial poses are represented as array of 3 positions and
    4 quaternion elements

    Parameters
    ----------
    s : array of floats
        path parameter (ns,)
    path : array of floats
        Cartesian path poses (ns, 7)
    squery : array of floats
        query path points (ni, )
    short : bool, optional
        if true, shortest rotation is taken

    Returns
    -------
    array of floats
        Cartesian path poses at query points (ni, 7)
    """
    s = vector(s)
    path = np.array(path)
    if not ismatrix(path, shape=(len(s), 7)):
        raise TypeError(f'Qpath must have dimension {(len(s), 7)}, but has {path.shape}')
    p = interpPath(s,path[:,:3], squery)
    Q = interpQuaternionPath(s, path[:,3:], squery, short=short)
    return np.hstack((p, Q))

def gradientPath(path, s):
    """Calculate gradient along path

    Parameters
    ----------
    path : array of floats
        path samples (nsamp, n)
    s : array of floats
        path parameter (nsamp,)

    Returns
    -------
    array of floats
        gradient along path (nsamp, n)
    """
    s = vector(s)
    path = np.array(path)
    if ismatrix(path) and path.shape[0]==len(s):
        return np.gradient(path, s, axis=0, edge_order = 1)
    else:
        raise TypeError(f'path and s must have same first dimension, but have shapes {path.shape} and {s.shape}')

def gradientQuaternionPath(path, s):
    """Calculate velocity along quaternion path

    Parameters
    ----------
    path : array of floats
        quaternion elements (nsamp, 4)
    s : array of floats
        path parameter (nsamp,)

    Returns
    -------
    array of floats
        gradient along quaternion path (nsamp, 3)
    """
    s = vector(s)
    path = np.array(path)
    if not ismatrix(path, shape=(len(s), 4)):
        raise TypeError(f'path must have dimension {(len(s), 4)}, but has {path.shape}')
    grad=np.gradient(path, s, axis=0)
    omega_q=2*(Quaternion.array(grad)*Quaternion.array(path).conj()).ndarray
    return omega_q[:,1:]

def gradientCartesianPath(path, s):
    """Calculate gradient along Cartesian path

    Poses are defined by position and quaternion.

    Parameters
    ----------
    path : array of floats
        Cartesian poses (nsamp, 7)
    s : array of floats
        path parameter (nsamp,)

    Returns
    -------
    array of floats
        gradient along Cartesian path (nsamp, 6)
    """
    s = vector(s)
    path = np.array(path)
    if not ismatrix(path, shape=(len(s), 7)):
        raise TypeError(f'path must have dimension {(len(s), 7)}, but has {path.shape}')
    v = gradientPath(path[:,:3], s)
    w = gradientQuaternionPath(path[:,3:], s)
    return np.hstack((v, w))

if __name__ == '__main__':
    from transformations import *
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

    import matplotlib.pyplot as plt

    t = np.linspace(0,2,num=201)

    # # Joint trajectories
    q0 = np.array((0,1,6,-3))
    q1 = np.array((1,2,3,4))
    q2 = np.array((2,3,-5,7))

    fig, ax = plt.subplots(3,3,num=1,figsize=(8, 8))
    qt, qdt, qddt = jline(q1,q2,t)
    ax[0, 0].plot(t, qt)
    ax[0, 0].grid()
    ax[0, 0].set_title('Line')
    ax[1, 0].plot(t, qdt)
    gqt = np.gradient(qt, t, axis=0)
    ax[1, 0].plot(t, gqt, '--')
    ax[1, 0].grid()
    ax[1, 0].set_title('Velocity')
    ax[2, 0].plot(t, qddt)
    ax[2, 0].grid()
    ax[2, 0].set_title('Acceleration')

    qt, qdt, qddt = jtrap(q1,q2,t)
    ax[0, 1].plot(t, qt)
    ax[0, 1].grid()
    ax[0, 1].set_title('Trap')
    ax[1, 1].plot(t, qdt)
    gqt = np.gradient(qt, t, axis=0)
    ax[1, 1].plot(t, gqt, '--')
    ax[1, 1].grid()
    ax[1, 1].set_title('Velocity')
    ax[2, 1].plot(t, qddt)
    ax[2, 1].grid()
    ax[2, 1].set_title('Acceleration')

    qt, qdt, qddt = jtraj(q1,q2,t)
    ax[0, 2].plot(t, qt)
    ax[0, 2].grid()
    ax[0, 2].set_title('Traj')
    ax[1, 2].plot(t, qdt)
    gqt = np.gradient(qt, t, axis=0)
    ax[1, 2].plot(t, gqt, '--')
    ax[1, 2].grid()
    ax[1, 2].set_title('Velocity')
    ax[2, 2].plot(t, qddt)
    ax[2, 2].grid()
    ax[2, 2].set_title('Acceleration')

    # Cartesian trajectories
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

    x0 = np.array([ 0.0349, -0.4928,  0.6526,  0.0681,  0.7280, -0.6782,  0.0730])
    x1 = np.array([ 0.4941,  0.0000,  0.6526,  0.0000, -0.9950,  0.0000, -0.0998])
    xt, xdt, xddt = ctrap(x0,x1,t)
    xt1, xdt1, xddt1 = ctrap(x0,x1,t)
    xt2, xdt2, xddt2 = ctraj(x0,x1,t)
    fig, ax = plt.subplots(3, 2, num=2, figsize=(8, 8))
    ax[0, 0].plot(t, xt[:,:3] )
    ax[0, 0].plot(t, xt1[:,:3], '--')
    ax[0, 0].plot(t, xt2[:,:3], ':')
    ax[0, 0].grid()
    ax[0, 0].set_title('$p$')
    ax[1, 0].plot(t, xdt[:,:3] )
    ax[1, 0].plot(t, xdt1[:,:3], '--')
    ax[1, 0].plot(t, xdt2[:,:3], ':')
    ax[1, 0].grid()
    ax[1, 0].set_title('$\dot p$')
    ax[2, 0].plot(t, xddt[:,:3] )
    ax[2, 0].plot(t, xddt1[:,:3], '--')
    ax[2, 0].plot(t, xddt2[:,:3], ':')
    ax[2, 0].grid()
    ax[2, 0].set_title('$\ddot p$')

    ax[0, 1].plot(t, xt[:,3:] )
    ax[0, 1].plot(t, xt1[:,3:], '--' )
    ax[0, 1].plot(t, xt2[:,3:], ':')
    ax[0, 1].grid()
    ax[0, 1].set_title('$Q$')
    ax[1, 1].plot(t, xdt[:,3:] )
    ax[1, 1].plot(t, xdt1[:,3:], '--' )
    ax[1, 1].plot(t, xdt2[:,3:], ':')
    ax[1, 1].grid()
    ax[1, 1].set_title('$\omega$')
    ax[2, 1].plot(t, xddt[:,3:] )
    ax[2, 1].plot(t, xddt1[:,3:], '--' )
    ax[2, 1].plot(t, xddt2[:,3:], ':')
    ax[2, 1].grid()
    ax[2, 1].set_title('$\dot\omega$')

    # Trajectory - Multi point interpolation
    t = np.linspace(0,4,num=401)
    ti, _, _ = jtraj(0, 2, t)
    s = [0, 1, 1.75, 2]
    xx = np.vstack((x0,x1,x2,x3))

    xt = interpCartesianPath(s,xx,ti)
    xdt = gradientCartesianPath(xt,t)
    xddt = gradientPath(xdt,t)

    fig, ax = plt.subplots(3,2,num=3,figsize=(8, 8))
    ax[0, 0].plot(t, xt[:,:3] )
    ax[0, 0].grid()
    ax[0, 0].set_title('$p$')
    ax[1, 0].plot(t, xdt[:,:3] )
    ax[1, 0].grid()
    ax[1, 0].set_title('$\dot p$')
    ax[2, 0].plot(t, xddt[:,:3] )
    ax[2, 0].grid()
    ax[2, 0].set_title('$\ddot p$')

    ax[0, 1].plot(t, xt[:,3:] )
    ax[0, 1].grid()
    ax[0, 1].set_title('$Q$')
    ax[1, 1].plot(t, xdt[:,3:] )
    ax[1, 1].grid()
    ax[1, 1].set_title('$\omega$')
    ax[2, 1].plot(t, xddt[:,3:] )
    ax[2, 1].grid()
    ax[2, 1].set_title('$\dot\omega$')


    # Cartesian arc trajectories
    p0 = np.array([0, 1, 3])
    p1 = np.array([1, 4, -1])
    pC = np.array([-1, 1, 1])
    Q0=rot_x(0, unit='deg')
    Q1=rot_x(60, unit='deg')
    Q2=rot_y(30, unit='deg')
    x0 = rp2t(Q=Q0, p=p0)
    x1 = rp2t(Q=Q1, p=p1)

    xt, xdt, xddt = carctraj(x0,x1,pC,t)
    fig, ax = plt.subplots(3, 2, num=4, figsize=(8, 8))
    ax[0, 0].plot(t, xt[:,:3] )
    ax[0, 0].grid()
    ax[0, 0].set_title('$p$')
    ax[1, 0].plot(t, xdt[:,:3] )
    ax[1, 0].grid()
    ax[1, 0].set_title('$\dot p$')
    ax[2, 0].plot(t, xddt[:,:3] )
    ax[2, 0].grid()
    ax[2, 0].set_title('$\ddot p$')

    ax[0, 1].plot(t, xt[:,3:] )
    ax[0, 1].grid()
    ax[0, 1].set_title('$Q$')
    ax[1, 1].plot(t, xdt[:,3:] )
    ax[1, 1].grid()
    ax[1, 1].set_title('$\omega$')
    ax[2, 1].plot(t, xddt[:,3:] )
    ax[2, 1].grid()
    ax[2, 1].set_title('$\dot\omega$')

    from mpl_toolkits import mplot3d
    fig = plt.figure(num=5)
    ax = plt.axes(projection="3d")
    ax.plot(xt[:,0], xt[:,1], xt[:,2])
    ax.grid()
    ax.set_aspect('equal')
    ax.set_title('$3D$')

    plt.show()
