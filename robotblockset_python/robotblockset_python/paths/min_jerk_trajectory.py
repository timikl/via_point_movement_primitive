import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
#%matplotlib inline

import math
import scipy.optimize
from scipy.interpolate import interp1d

from robotblockset_python.paths.trajectories import interpQuaternionPath

# trj, psg = min_jerk(pos, dur, vel, acc, psg)
# 
# Compute minimum-jerk trajectory through specified points
#
# INPUTS:
# pos: NxD array with the D-dimensional coordinates of N points
# dur: number of time steps (integer)
# vel: 2xD array with endpoint velocities, [] sets vel to 0
# acc: 2xD array with endpoint accelerations, [] sets acc to 0
# psg: (N-1)x1 array of via-point passage times (between 0 and dur);
#      [] causes optimization over the passage times
#
# OUTPUTS
# trj: dur x D array with the minimum-jerk trajectory
# psg: (N-1)x1 array of passage times
#
# This is an implementation of the algorithm described in:
#  Todorov, E. and Jordan, M. (1998) Smoothness maximization along
#  a predefined path accurately predicts the speed profiles of
#  complex arm movements. Journal of Neurophysiology 80(2): 696-714
# The paper is available online at www.cogsci.ucsd.edu/~todorov

# Copyright (C) Emanuel Todorov, 1998-2006
# Python implementation by Dogancan Kebude
#https://github.com/dkebude/py-min-jerk/blob/master/min_jerk.py

def min_jerk(pos=None, dur=None, vel=None, acc=None, psg=None):

    N = pos.shape[0]# number of point
    D = pos.shape[1]# dimensionality

    if not vel:
        vel = np.zeros((2,D))# default endpoint vel is 0
    if not acc:
        acc = np.zeros((2,D))# default endpoint acc is 0

    t0 = np.array([[0],[dur]])

    if not psg:# passage times unknown, optimize
        if N > 2:
            psg = np.arange(dur/(N-1), dur-dur/(N-1)+1, dur/(N-1)).T
            func = lambda psg_: mjCOST(psg_, pos, vel, acc, t0)
            psg = scipy.optimize.fmin(func = func, x0 = psg)
        else:
            psg = []

    #print(psg)
    trj = mjTRJ(psg, pos, vel, acc, t0, dur)

    return trj, psg

################################################################
###### Compute jerk cost
################################################################

def mjCOST(t, x, v0, a0, t0):
     
    N = x.shape[0]
    D = x.shape[1]
    #N = max(x.shape)
    #D = min(x.shape)

    v, a = mjVelAcc(t, x, v0, a0, t0)
    aa   = np.concatenate(([a0[0][:]], a, [a0[1][:]]), axis = 0)
    aa0  = aa[0:N-1][:]
    aa1  = aa[1:N][:]
    vv   = np.concatenate(([v0[0][:]], v, [v0[1][:]]), axis = 0)
    vv0  = vv[0:N-1][:]
    vv1  = vv[1:N][:]
    tt   = np.concatenate((t0[0]   , t, t0[1]   ), axis = 0)
    T    = np.diff(tt)[np.newaxis].T*np.ones((1,D))
    xx0  = x[0:N-1][:]
    xx1  = x[1:N][:]

    j=3*(3*aa0**2*T**4-2*aa0*aa1*T**4+3*aa1**2*T**4+24*aa0*T**3*vv0- \
         16*aa1*T**3*vv0 + 64*T**2*vv0**2 + 16*aa0*T**3*vv1 - \
         24*aa1*T**3*vv1 + 112*T**2*vv0*vv1 + 64*T**2*vv1**2 + \
         40*aa0*T**2*xx0 - 40*aa1*T**2*xx0 + 240*T*vv0*xx0 + \
         240*T*vv1*xx0 + 240*xx0**2 - 40*aa0*T**2*xx1 + 40*aa1*T**2*xx1- \
         240*T*vv0*xx1 - 240*T*vv1*xx1 - 480*xx0*xx1 + 240*xx1**2)/T**5

    J = sum(sum(abs(j)));

    return J

################################################################
###### Compute trajectory
################################################################

def mjTRJ(tx, x, v0, a0, t0, P):

    #N = max(x.shape)
    #D = min(x.shape)
    N = x.shape[0]
    D = x.shape[1]
    
    X_list = []

    if len(tx) > 0:
        v, a = mjVelAcc(tx, x, v0, a0, t0)
        aa   = np.concatenate(([a0[0][:]],  a, [a0[1][:]]), axis = 0)
        vv   = np.concatenate(([v0[0][:]],  v, [v0[1][:]]), axis = 0)
        tt   = np.concatenate((t0[0]   , tx, t0[1]   ), axis = 0)
    else:
        aa = a0
        vv = v0
        tt = t0

    ii = 0
    for i in range(1,int(P)+1):
        t = (i-1)/(P-1)*(t0[1]-t0[0]) + t0[0]
        if t > tt[ii+1]:
            ii = ii+1
        T = (tt[ii+1]-tt[ii])*np.ones((1,D))
        t = (t-tt[ii])*np.ones((1,D))
        aa0 = aa[ii][:]
        aa1 = aa[ii+1][:]
        vv0 = vv[ii][:]
        vv1 = vv[ii+1][:]
        xx0 = x[ii][:]
        xx1 = x[ii+1][:]

        tmp = aa0*t**2/2 + t*vv0 + xx0 + t**4*(3*aa0*T**2/2 - aa1*T**2 + \
              8*T*vv0 + 7*T*vv1 + 15*xx0 - 15*xx1)/T**4 + \
              t**5*(-(aa0*T**2)/2 + aa1*T**2/2 - 3*T*vv0 - 3*T*vv1 - 6*xx0+ \
                    6*xx1)/T**5 + t**3*(-3*aa0*T**2/2 + aa1*T**2/2 - 6*T*vv0 - \
                                        4*T*vv1 - 10*xx0 + 10*xx1)/T**3
        X_list.append(tmp)

    X = np.concatenate(X_list)

    return X

################################################################
###### Compute intermediate velocities and accelerations
################################################################

def mjVelAcc(t, x, v0, a0, t0):

    #N = max(x.shape)
    #D = min(x.shape)
    N = x.shape[0]
    D = x.shape[1]
    
    mat = np.zeros((2*N-4,2*N-4))
    vec = np.zeros((2*N-4,D))
    tt = np.concatenate((t0[0], t, t0[1]), axis = 0)

    for i in range(1, 2*N-4+1, 2):

        ii = int(math.ceil(i/2.0))
        T0 = tt[ii]-tt[ii-1]
        T1 = tt[ii+1]-tt[ii]

        tmp = [-6/T0, -48/T0**2, 18*(1/T0+1/T1), \
               72*(1/T1**2-1/T0**2), -6/T1, 48/T1**2]

        if i == 1:
            le = 0
        else:
            le = -2

        if i == 2*N-5:
            ri = 1
        else:
            ri = 3

        mat[i-1][i+le-1:i+ri] = tmp[3+le-1:3+ri]
        vec[i-1][:] = 120*(x[ii-1][:]-x[ii][:])/T0**3 \
                      + 120*(x[ii+1][:]-x[ii][:])/T1**3

    for i in range(2, 2*N-4+1, 2):

        ii = int(math.ceil(i/2.0))
        T0 = tt[ii]-tt[ii-1]
        T1 = tt[ii+1]-tt[ii]

        tmp = [48/T0**2, 336/T0**3, 72*(1/T1**2-1/T0**2), \
               384*(1/T1**3+1/T0**3), -48/T1**2, 336/T1**3]

        if i == 2:
            le = -1
        else:
            le = -3

        if i == 2*N-4:
            ri = 0
        else:
            ri = 2

        mat[i-1][i+le-1:i+ri] = tmp[4+le-1:4+ri]
        vec[i-1][:] = 720*(x[ii][:]-x[ii-1][:])/T0**4 \
                      + 720*(x[ii+1][:]-x[ii][:])/T1**4

    T0 = tt[1] - tt[0]
    T1 = tt[N-1]-tt[N-2]
    vec[0][:] = vec[0][:] +  6/T0*a0[0][:]    +  48/T0**2*v0[0][:]
    vec[1][:] = vec[1][:] - 48/T0**2*a0[0][:] - 336/T0**3*v0[0][:]
    vec[2*N-6][:] = vec[2*N-6][:] +  6/T1*a0[1][:]    -  48/T1**2*v0[1][:]
    vec[2*N-5][:] = vec[2*N-5][:] + 48/T1**2*a0[1][:] - 336/T1**3*v0[1][:]

    avav = inv(mat).dot(vec)
    a = avav[0:2*N-4:2][:]
    v = avav[1:2*N-4:2][:]

    return v, a

def interp_to_size(x_arr, new_size):
    # Deepcopy myb ?
    if x_arr.shape[0]>new_size:
        #print("interpolating")
        out_interp = np.zeros((new_size, x_arr.shape[1]))
        
        for dof in range(x_arr.shape[1]):
            
            x = x_arr[:, dof]
            y = np.linspace(0, 1, x_arr.shape[0])
            fn = interp1d(y, x, bounds_error=False)
        
            # We want to get out 5 points
            x_interp = np.linspace(0, 1, new_size)
            
            out_interp[:,dof] = fn(x_interp)
    
        x_arr = out_interp
    return x_arr

def generate_min_jerk(x_arr, move_time, tsamp, new_size, control_space_for_min_jerk = 'x', return_values = 'x'):
    
    assert control_space_for_min_jerk in ['x', 'q'] # DO you want to generate a min jerk trajectory in x or q space ?
    assert return_values in ['x', 'q'] # Do you want to return qs or xs
        
    n_dur = int(move_time/tsamp) # 100
    
    new_size = np.min((new_size, x_arr.shape[0])) # Interp to 5 values in between. Then make a larger array.
    
    assert return_values == 'q'
    assert control_space_for_min_jerk == 'q'

    # Do calc
    assert x_arr.shape[1] == 7
    if control_space_for_min_jerk == 'x':

        #print("Cartesian space")
        x_pos = x_arr[:,:3]
        x_quat = x_arr[:,3:]

        # Interpolate the x_pos
        x_pos = interp_to_size(x_pos, new_size)
        # Get minimum jerk for position
        traj_pos, psg_pos = min_jerk(pos=x_pos, dur=n_dur, vel=None, acc=None, psg=None)
    
        # We first use quaternion interpolation and then do a minimum jerk trajectory over a subset of points
        
        #s = np.linspace(0,move_time,n_dur)
    
        
        s = np.linspace(0,move_time+tsamp,x_quat.shape[0])
        #time=np.arange(0,move_time+tsamp,tsamp)
        
        #time=np.arange(0,move_time+tsamp,tsamp)  # query points
        time = np.linspace(0,move_time, new_size)
        
        #print(s.shape, x_quat.shape, time.shape, new_size)
        x_quat = interpQuaternionPath(s, x_quat, time, short=True)

        traj_quat, psg_quat = min_jerk(pos=x_quat, dur=n_dur, vel=None, acc=None, psg=None)
        
        for i in range(1,traj_quat.shape[0]):
            prev_q = traj_quat[i-1, :]
            nq = traj_quat[i, :]
            
            d = np.dot(prev_q, nq)
            if d<0:
                traj_quat[i,:] = -nq
        
        #print("X_pos sh:", traj_pos.shape)
        #print("X_quat sh:", traj_quat.shape)

    
        traj = np.hstack((traj_pos, traj_quat))
        return traj, v, a, psg_pos

    elif control_space_for_min_jerk == 'q':
        #print("JointSpace")
        # Make sure max 5 points
        x_arr = interp_to_size(x_arr, new_size)
        

        #print("Dimension of in array:", x_arr.shape)
        traj, psg = min_jerk(pos=x_arr, dur=n_dur, vel=None, acc=None, psg=None)
    else:
        print("error, strategy not supported. minjerktrajectorygenerator")
        
        
    if (control_space_for_min_jerk == 'q') and (return_values == 'q'):
        return traj, psg # We dont have to change anything
    elif (control_space_for_min_jerk == 'q') and (return_values == 'x'):
        raise Exception("Not impl")
    elif (control_space_for_min_jerk == 'x') and (return_values == 'q'):
        raise Exception("Not impl")
    elif (control_space_for_min_jerk == 'x') and (return_values == 'x'):
        return traj, psg # We dont have to change anything


if __name__ == '__main__':
    # Get input
    x_arr = np.zeros((100, 7))

    for i in range(0,x_arr.shape[1]):
        x_arr[:,i] = np.linspace(0,i+1,x_arr.shape[0])
        x_arr[:,i] = np.cos(x_arr[:,i])

    move_time = 1
    tsamp = 0.01

    n_dur = int(move_time/tsamp) # 100
    # Tells you how many points you want to get out

    new_size = 3 # the number of points to interpolate to (in between when calculating optimal "passage points")

    _control_strategy = 'JointImpedance'

    traj, psg = generate_min_jerk(x_arr = x_arr, move_time = move_time, tsamp= tsamp, new_size = new_size, _control_strategy = _control_strategy)



    t = np.linspace(0,move_time, traj.shape[0])
    plt.plot(t, traj)
    #plt.plot(time, traj_vel[0,:])

    #plt.title("Minimum jerk trajectory")
    #plt.xlabel("Time [s]")
    #plt.ylabel("Angle [deg] and angular velocity [deg/s]")
    #plt.legend(['pos', 'vel'])
    #plt.show()
    #print(traj.shape)

    # plot passage points
    plt.plot(psg*tsamp, np.ones_like(psg), 'ko')
