import enum
from class_polynomial import polynomial
import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore


if __name__ == "__main__":

    test_trajectory = polynomial([0, 0, 0], [0, 60, 0], [0, 0, 0], [10, 10, 10], [0, 0, 0], [0, 0, 0], 0.5, 0.01)

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
        

    traj_plot.set_ylabel('Position')
    vel_plot.set_ylabel('Velocity')
    vel_plot.set_xlabel('Time')
    acc_plot.set_ylabel('Acceleration')
    acc_plot.set_xlabel('Time')
    jerk_plot.set_ylabel('Jerk')
    jerk_plot.set_xlabel('Time')

    #print(f"{Fore.LIGHTCYAN_EX}i: {test_trajectory.trajectory} | type: {type(test_trajectory.trajectory)} | len: {len(test_trajectory.trajectory)}")
    #print(f"{Fore.LIGHTMAGENTA_EX}t: {test_trajectory.time_split} | type: {type(test_trajectory.time_split)} | len: {len(test_trajectory.time_split)}")

    ax = plt.axes(projection="3d")
    for i in test_trajectory.jerk:
        ax.scatter3D (i[0],i[1],i[2])
    plt.show()
    

