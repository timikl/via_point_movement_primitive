import enum
from class_polynomial import Polynomial
import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore


if __name__ == "__main__":

    test_trajectory = Polynomial([0, 0, 0], [0, 0, 0], [0, 0, 0], [10, 10, 10], [0, 0, 0], [0, 0, 0], 100, 0.1)

    # Create two subplots
    fig, (traj_plot, vel_plot, acc_plot, jerk_plot) = plt.subplots(4, 1)
    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    for index, i in enumerate(time):
        
        #prints velocity and acceleration
        print(f"{Fore.BLUE}||{index}|| {Fore.LIGHTBLACK_EX} Pose: {test_trajectory.trajectory[index]}|{Fore.GREEN}Vel : {test_trajectory.velocity[index]} m/s | {Fore.RED} Acc: {test_trajectory.acceleration[index]} | {Fore.CYAN} Jerk: {test_trajectory.jerk[index]}")
        
""""
    traj_plot.set_ylabel('Position')
    vel_plot.set_ylabel('Velocity')
    vel_plot.set_xlabel('Time')
    acc_plot.set_ylabel('Acceleration')
    acc_plot.set_xlabel('Time')
    jerk_plot.set_ylabel('Jerk')
    jerk_plot.set_xlabel('Time')
"""
    #print(f"{Fore.LIGHTCYAN_EX}i: {test_trajectory.trajectory} | type: {type(test_trajectory.trajectory)} | len: {len(test_trajectory.trajectory)}")
    #print(f"{Fore.LIGHTMAGENTA_EX}t: {test_trajectory.time_split} | type: {type(test_trajectory.time_split)} | len: {len(test_trajectory.time_split)}")



jerk_plot.plot(test_trajectory.time_split, np.abs(np.asarray(test_trajectory.jerk)[:]))
acc_plot.plot(test_trajectory.time_split, np.abs(np.asarray(test_trajectory.acceleration)[:]))
vel_plot.plot(test_trajectory.time_split, np.abs(np.asarray(test_trajectory.velocity)[:]))

plt.show()
"""
    #Trajectory plot
    aux_traj = []
    for i in test_trajectory.trajectory:
        #traj_plot.scatter(test_trajectory.time_split,i[1])
        aux_traj.append(i[0])

    traj_plot.scatter(test_trajectory.time_split, aux_traj)

    #Velocity plot
    aux_vel = []
    for j in test_trajectory.velocity:
        #vel_plot.scatter(j[0], j[1])
        aux_vel.append(j[0])

    vel_plot.scatter(test_trajectory.time_split[:], aux_vel)

    #Acceleration plot
    aux_acc = []
    for z in test_trajectory.acceleration:
        #acc_plot.scatter(z[0], z[1])
        aux_acc.append(z[0])
    
    acc_plot.scatter(test_trajectory.time_split, aux_acc)

    #Jerk plot
    aux_jerk_size = []
    
    for k in test_trajectory.jerk:
        #acc_plot.scatter(z[0], z[1])

        aux_jerk_size.append(np.linalg.norm(k))

    #print((len(aux_jerk), len(test_trajectory.time_split)))

    jerk_plot.scatter(test_trajectory.time_split, aux_jerk_size)
"""



