from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.grippers import VariableStiffnessGripper

from test_controllers import test_controller_cmove, test_controller_cmovefor, test_controller_cpath, test_controller_jmove, test_sequence

r = panda_ros('panda_2', start_controller = 'position_joint_trajectory_controller', init_frankadesk_gripper_TCP = True)
r.SetCollisionBehavior(F=50, T= 20, tq = 30)
r.error_recovery()
sc_grp = VariableStiffnessGripper()
r.gripper = sc_grp
r.gripper.open()

r.GetState()
r.JMove(r.q, 0.1)

if 1:
    test_sequence(r, controller = 'JointPositionTrajectory', reset_targets_in_between=False)
