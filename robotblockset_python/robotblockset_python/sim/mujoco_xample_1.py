# Example using "mujoco_haptix"
#
# Note:  Open Haptix model (for this example use robot_demo.xml)
#
# More information about MuJoCo. on http://www.haptix.org/book/scn.html 

import numpy as np
import mujoco_haptix as haptix

# Path to scn dll
path = "d:\\Test\\Python\\Robot\\mjhaptix_user.dll"
# path = "d:\\Leon\\MATLAB\\toolbox\\mjhaptix\\apicpp\\mjhaptix_user.dll"

# Select native API
scn = haptix.mjInterface()
try:
    if scn.mj_connect(None)==0:
        np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
        info=scn.mj_info()
        print('Timestep: %.4f' %info.timestep)

        mocap=scn.mj_get_mocap()
        mocap_pos=np.array(mocap.pos[:mocap.nmocap])
        print('All mocaps positions:\n',mocap_pos[:mocap.nmocap])
        print('Robot base: ',mocap.pos[0][:3])

        bb=scn.mj_get_body()
        body_pos=np.array(bb.pos[:bb.nbody])
        body_mat=np.array(bb.mat[:bb.nbody]).reshape((-1,3,3))
        print('Body 0 name: ',scn.mj_id2name('body',2))
        print('Body 0 pos:', body_pos[2][:3])
        print('Body 0 rot:', body_mat[2])

        # New robot base position (using tuple)
        mocap.pos[0] = (0.1, 0, 0.3)
        scn.mj_set_mocap(mocap)

        # Current control (desired joint positions)
        u = scn.mj_get_control()
        print('Robot desired joint positions: ',u.ctrl[:u.nu])

        # Current joint positions
        st = scn.mj_get_state()
        q = st.qpos[:st.nq]
        print('Robot actual  joint positions: ',q)

        # New random joint position (using numpy array)
        u.ctrl[:u.nu]=np.random.rand(u.nu)*2-1
        scn.mj_set_control(u)


        sen=scn.mj_get_sensor()
        print('Sensor:',sen.sensordata[:sen.nsensordata])

        print('Body name: ',scn.mj_id2name('body',1))
        print('Site ID: ',scn.mj_name2id('site','Tool'))
except Exception as inst:
    raise inst
finally:
    scn.mj_close()
