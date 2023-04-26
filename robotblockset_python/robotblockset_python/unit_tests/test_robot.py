import numpy as np
import matplotlib.pyplot as plt

from robots_haptix import panda_haptix
from robots_roboworks import panda_roboworks, lwr_roboworks
from transformations import map_pose, rot_x
from robots import robot, isrobot
from grippers import panda_gripper_roboworks

np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

# Callback for Motion check
def CheckQ(r):
    if isrobot(r):
        data = r.GetUserdata()
        if r.q[0]>data['q1m']:
            print('Check limit achieved: ',r.q)
            return 1
        else:
            return 0

it = 0
nn = 10000
tt = np.empty(nn)
qt = np.empty((nn,7)) 
qdt = np.empty((nn,7))
rqt = np.empty((nn,7)) 
rqdt = np.empty((nn,7))
xt = np.empty((nn,7))
vt = np.empty((nn,6))
rxt = np.empty((nn,7))
rvt = np.empty((nn,6))

# Callback for Update
def PrintQ(r):
    if isrobot(r):
        global it
        tt[it] = r.Time
        qt[it] = r._actual.q
        qdt[it] = r._actual.qdot
        rqt[it] = r._command.q
        rqdt[it] = r._command.qdot
        xt[it] = r._actual.x
        vt[it] = r._actual.v
        rxt[it] = r._command.x
        rvt[it] = r._command.v
        it += 1

# Robot selection
r = panda_haptix()
g = panda_gripper_roboworks()

r.SetGripper(g)
g.Open()

print('Distance to joint limits: ', r.DistToJointLimits()[0] )
print('Time 1: %.3f'%r.Time, r.q)
r.JMove(r.q_home,2)
print('Time 2: %.3f'%r.Time, r.q)
q1 = r.q_home+0
q1[0] = -1.5
r.JMove(q1,3)
print('Time 3: %.3f'%r.Time, r.q)
x0 = map_pose(p=[0.4, 0, 0.6],  Q=rot_x(180, unit='deg'))
print('Ref x:', x0)
try:
    r.SetMotionCheckCallback(CheckQ)
    data = {'q1m': 0.5}
    r.SetUserData(data)
    r.SetCaptureCallback(PrintQ)
    r.ResetTime
    r.StartCapture()
    r.CMove(x0, 2)
    print('Fin x:', r.GetPose())
    print('Time 4: %.3f'%r.Time, r.q)

    q1[0] = +1.5
    r.EnableMotionCheck()
    r.JMove(q1,3)
    print('Time 3: %.3f'%r.Time, r.q)
    r.DisableMotionCheck()
    # r.ResetTaskTarget()
    r.SetMocapPose('Target',x0)
    r.CLine(x0, 1)
    r.SetObject(map_pose(p=[0.4,0,0.6], Q=rot_x(np.pi)))
    r.CMoveFor([0,0.1,0], 1)
    r.OMove([0,0.15,0], 1)
    r.HaptixMessage('Arc')
    r.CArc([0.15,0,0],[0,0,0],5, task_space='Object')
    print('Task distance:', r.TaskDistance([0,0,0], task_space='Object', state='Command'))

finally:
    r.StopCapture    

print('Done')

fig, ax = plt.subplots(2,3,num=1,figsize=(9, 6))
ax[0, 0].plot(tt[:it], qt[:it], '-')
ax[0, 0].plot(tt[:it], rqt[:it], '--')
ax[0, 0].grid()
ax[0, 0].set_title('$q$')
ax[1, 0].plot(tt[:it], qdt[:it], '-')
ax[1, 0].plot(tt[:it], rqdt[:it], '--')
ax[1, 0].grid()
ax[1, 0].set_title('$\dot q$')

ax[0, 1].plot(tt[:it], xt[:it,:3] )
ax[0, 1].plot(tt[:it], rxt[:it,:3], '--' )
ax[0, 1].grid()
ax[0, 1].set_title('$p$')
ax[1, 1].plot(tt[:it], vt[:it,:3] )
ax[1, 1].plot(tt[:it], rvt[:it,:3], '--' )
ax[1, 1].grid()
ax[1, 1].set_title('$\dot p$')

ax[0, 2].plot(tt[:it], xt[:it,3:] )
ax[0, 2].plot(tt[:it], rxt[:it,3:], '--' )
ax[0, 2].grid()
ax[0, 2].set_title('$Q$')
ax[0, 2].legend(['$s$','$v_1$','$v_2$','$v_3$'])
ax[1, 2].plot(tt[:it], vt[:it,3:] )
ax[1, 2].plot(tt[:it], rvt[:it,3:], '--' )
ax[1, 2].grid()
ax[1, 2].set_title('$\omega$')

plt.show()


