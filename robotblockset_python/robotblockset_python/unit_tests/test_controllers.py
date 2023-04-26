import numpy as np
import copy
import time
from disassembly_pipeline.saved_positions import p1_q1_init,p2_q1_init

valid_controllers = ['CartesianImpedance', 'JointPositionTrajectory']


def test_controller_cmove(robot, controller = 'CartesianImpedance', reset_targets_in_between=False, wait_until_stopped = False):
    assert robot.Name == 'panda_2'
    assert controller in valid_controllers
    #print("P1", robot.GetPose(state = 'Commanded', kinematics = 'Calculated'))
    
    robot.ResetCurrentTarget()
    if robot._control_strategy != controller:
        robot.Switch_controller(start_controller = controller)
        
    #print("P2", robot.GetPose(state = 'Commanded', kinematics = 'Calculated'))

    
    r = robot
    r.error_recovery()
    r.JMove(p2_q1_init,3)
    if reset_targets_in_between:r.ResetCurrentTarget()
    if wait_until_stopped: r.WaitUntilStopped()
    
    #r.error_recovery()
    #r.ResetCurrentTarget()
    #r.Stop()
    #r.CMoveFor([0.1,0,0],1)
    #time.sleep(0.5)
    #r.CMoveFor([-0.1,0,0],1)
    #r.GetState()
    #x0 = r.GetPose(short = True, kinematics = 'Calculated', state = 'Commanded', task_space= 'World', changing_stiffness = False)
    x1 = r.x
    x2 = copy.deepcopy(x1)
    x2[0] +=0.05
    x3 = copy.deepcopy(x2)
    x3[2] +=0.03
    x4 = copy.deepcopy(x3)
    x4[2] +=0.02
    x5 = copy.deepcopy(x4)
    x5[0]+=0.02
    
    

    path = np.stack((x1, x2, x3, x4, x5))
    #path = np.stack((cur_x, x_above_new))
        
    tpath = (3)
    tpath = r.tsamp * round( tpath / r.tsamp )  
    time.sleep(0.1)
    if reset_targets_in_between:r.ResetCurrentTarget()
    
    r.CMove(x1,2)
    #print("P3", robot.GetPose(state = 'Commanded', kinematics = 'Calculated'))
        
    if wait_until_stopped: r.WaitUntilStopped()
    r.GetState()
    if reset_targets_in_between:r.ResetCurrentTarget()
    
    r.CMove(x2,1)
    
    if wait_until_stopped: r.WaitUntilStopped()
    if reset_targets_in_between:r.ResetCurrentTarget()

    r.CMove(x3,1)
    
    if wait_until_stopped: r.WaitUntilStopped()
    if reset_targets_in_between:r.ResetCurrentTarget()
    
    r.CMove(x4,1)
    if wait_until_stopped: r.WaitUntilStopped()
    if reset_targets_in_between:r.ResetCurrentTarget()

    r.CMove(x5,1)
    
def test_controller_cmovefor(robot, controller =  'JointPositionTrajectory', reset_targets_in_between=False, task_space = 'World'):
    assert robot.Name in  ['panda_1', 'panda_2']
    assert controller in valid_controllers
    assert task_space in ['World', 'Tool']

    if robot._control_strategy != controller:
        robot.Switch_controller(start_controller = controller)
        
    r = robot

    r.error_recovery()
    
    if robot.Name == 'panda_1':
        x1 = [ 0.730143, -0.134029,  0.021748,  0.043824,  0.998457,  0.033707, -0.005174]
    elif robot.Name == 'panda_2':
        x1 = [ 0.371415, -0.082081,  0.359387,  0.009969,  0.999726,  0.009393, -0.018989]
    
    
    r.CMove(x1,2)
    q=np.array(r.q);
    print('q1=',q)
    if reset_targets_in_between:r.ResetCurrentTarget()
    
    r.CMoveFor([0,0,0.05],1 , task_space = task_space)
    q=np.array(r.q);
    print('q2=',q)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.CMoveFor([0.05,0,0.0],1, task_space = task_space)
    q=np.array(r.q);
    print('q3=',q)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.CMoveFor([0,0.05,0.0],1, task_space = task_space)
    q=np.array(r.q);
    print('q4=',q)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.CMoveFor([0,0,0.15],2, task_space = task_space)
    q=np.array(r.q);
    print('q5=',q)
    if reset_targets_in_between:r.ResetCurrentTarget()


def test_controller_cpath(robot, controller = 'JointPositionTrajectory', reset_targets_in_between = False, cpath_version = 'new'):
    
    assert robot.Name == 'panda_2'
    assert controller in ['JointPositionTrajectory']
    assert cpath_version in ['new', 'old']
    
    
    if robot._control_strategy != controller:
        robot.Switch_controller(start_controller = controller)
    r = robot

    
    r.error_recovery()
    r.JMove(p2_q1_init,3)
    #time.sleep(1)
    
    #r.error_recovery()
    #r.ResetCurrentTarget()
    #r.Stop()
    #r.CMoveFor([0.1,0,0],1)
    #time.sleep(0.5)
    #r.CMoveFor([-0.1,0,0],1)
    #r.GetState()
    #x0 = r.GetPose(short = True, kinematics = 'Calculated', state = 'Commanded', task_space= 'World', changing_stiffness = False)
    if reset_targets_in_between:r.ResetCurrentTarget()
    
    x1 = (0.369591, -0.080424,  0.357184,  0.019589,  0.999761,  0.005347,  0.008147)
    x2 = ( 0.425983,  0.041972,  0.366552,  0.219733, -0.961441,  0.165306,  0.00474 )
    x3 = ( 0.473266,  0.135888,  0.410473,  0.333188, -0.929583,  0.156191, -0.021577)
    
    #x1 = [ 0.369865, -0.080619,  0.357659,  0.020027,  0.999747,  0.005516,  0.008616]
    #x2 = [ 0.459865, -0.040619,  0.357659,  0.020027,  0.999747,  0.005516,  0.008616]
    #x3 = [ 0.59865, 0.080619,  0.357659,  0.020027,  0.999747,  0.005516,  0.008616]

    #x2 =  [ 0.424498, -0.294479,  0.320415,  0.238456, -0.807969,  0.506779, -0.183031]
    #x3 =  [ 0.434936, -0.363476,  0.408667,  0.270005, -0.628341,  0.694495, -0.223521]
    #x4 = [ 0.49305 ,  0.359311,  0.477014,  0.452561, -0.851128, -0.216099,  0.155147]
    #x5  =[ 0.290369, -0.082945,  0.438783,  0.032518,  0.94479 , -0.208635, -0.250572]
    #x1 = copy.deepcopy(r.x)
    #x2 = copy.deepcopy(x1)
    #x2[0] +=0.1
    #x3 = copy.deepcopy(x2)
    #x3[1] +=0.14
    #x4 = copy.deepcopy(x3)
    #x4[2] +=0.14
    #x5 = copy.deepcopy(x4)
    #x5[0]+=0.07


    path = np.stack((x1, x2, x3))
    #path = np.stack((cur_x, x_above_new))
        
    tpath = (3)
    tpath = r.tsamp * round( tpath / r.tsamp )  
    time.sleep(0.1)
    if cpath_version == 'new':
        out  = r.CPath_new(path=path, t = tpath)
    else:
        r.CPath(path = path, t = tpath)
    
def test_controller_jmove(robot, controller = 'CartesianImpedance', reset_targets_in_between=False):
    assert robot.Name == 'panda_1'
    assert controller in valid_controllers
    if robot._control_strategy != controller:
        robot.Switch_controller(start_controller = controller)
    
    r = robot
    r.ResetCurrentTarget()
    r.error_recovery()
    q1= [-0.100638,  0.564746, -0.148691, -2.192536, -0.023166,  2.765768,  0.512643]
    q2= [-0.086786,  0.441336, -0.165347, -2.221294, -0.03771,   2.672206,  0.514824]
    q3= [-0.086777,  0.541581, -0.148723, -2.036137, -0.022758,  2.586751,  0.521744]
    q4= [-0.009054,  0.521412, -0.14347,  -2.070228, -0.034463,  2.593469,  0.612924]
    q5= [ 0.010255,  0.241187, -0.163268, -2.039641, -0.06609,   2.285008,  0.612297]

    r.JMove(q1,2)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.JMove(q2,1)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.JMove(q3,1)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.JMove(q4,1)
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.JMove(q5,2)
    
def test_sequence_old(robot, controller = 'JointPositionTrajectory', reset_targets_in_between=False):
    assert robot.Name == 'panda_2'
    assert controller in ['JointPositionTrajectory']

    
    r = robot
    if r._control_strategy != controller:
        r.Switch_controller(start_controller = controller)
        
    r.error_recovery()
    
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.JMove(p2_q1_init,2)
    
    if reset_targets_in_between:r.ResetCurrentTarget()
    r.CMoveFor([0.1, 0, 0], 3)
    
    if reset_targets_in_between:r.ResetCurrentTarget()    
    r.CMove([0.5, 0, 0.4], 2)
    
    if reset_targets_in_between:r.ResetCurrentTarget()    
    r.JMove(p2_q1_init,2)


def test_sequence(robot, controller = 'JointPositionTrajectory', reset_targets_in_between=False, run_jmove = 0):
    """ 
    run_jmove : if 1, JMoves will be made. if 0, CMoves will be made"""
    
    xi = [[ 0.369433, -0.080202,  0.357421,  0.019583,  0.999765,  0.004982,  0.007807],
      [ 0.521948, -0.593131,  0.430825,  0.467941,  0.883663,  0.003904, -0.012471],
     [ 0.507369, -0.394651,  0.342391,  0.237413, -0.968218, -0.044226, -0.065057],
      [ 0.879963, -0.050888,  0.311979,  0.071863, -0.933693, -0.113959, -0.331764],
      [ 0.330667,  0.678711,  0.346429,  0.512128, -0.794463, -0.118391,  0.304199]]
    
    qi = [(-0.067853, -0.613377, -0.007917, -2.444159, -0.044061, 1.849221, 0.712784),
        (-0.403736, 0.308785, -0.312408, -1.663323, -1.000577, 2.368275, 0.709313),
        (-0.604992, 0.411232, -0.257847, -1.308227, 0.503089, 1.408834, -0.156696),
        (-0.065514, 0.633856, 0.023642, -1.311708, 0.424139, 2.557897, 0.208954),
        (0.345456, 0.646635, 0.672973, -1.423899, 0.993179, 2.019489, 0.911496)]
    
    
    assert robot.Name == 'panda_2'
    assert controller in ['JointPositionTrajectory']

    
    r = robot
    if r._control_strategy != controller:
        r.Switch_controller(start_controller = controller)
        
    r.error_recovery()
        #r.ResetCurrentTarget()
    #r.ResetCurrentTarget()

    n_iterations = 2

    run_jmove = 0
    if run_jmove:
        for it in range(0,n_iterations):
            for i, q in enumerate(qi):
                print(i, q)
                r.JMove(q, 3)
                #r.CMove(x, 3)
    else:
        # Run cmove
        for it in range(0,n_iterations):
            for i, x in enumerate(xi):
                print(i, x)
                #r.JMove(q, 3)
                r.CMove(x, 3, null_space_task = 'JointLimits')
                #r.CMove(x, 3)

    r.CMove(xi[0],3)
    
