from class_polynomial import polynomial
import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore


if __name__ == "__main__":
    
    test_trajectory = polynomial([0, 7], [0, 0], [0, 0], [10, 3.14], [0, 0], [0, 0], 10, 0.1)

    # Create two subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    print((time))

    ax1.set_ylabel('Position')
    ax2.set_ylabel('Velocity')
    ax2.set_xlabel('Time')
    ax3.set_ylabel('Acceleration')
    ax3.set_xlabel('Time')
    
    #print(f"{Fore.LIGHTCYAN_EX}i: {test_trajectory.trajectory} | type: {type(test_trajectory.trajectory)} | len: {len(test_trajectory.trajectory)}")
    #print(f"{Fore.LIGHTMAGENTA_EX}t: {test_trajectory.time_split} | type: {type(test_trajectory.time_split)} | len: {len(test_trajectory.time_split)}")
    
    aux_traj = []
    for i in test_trajectory.trajectory:
        #ax1.scatter(test_trajectory.time_split,i[1])
        aux_traj.append(i[0])

    ax1.scatter(test_trajectory.time_split, aux_traj)

    aux_vel = []
    for j in test_trajectory.velocity:
        #ax2.scatter(j[0], j[1])
        aux_vel.append(j[0])

    ax2.scatter(test_trajectory.time_split[:], aux_vel)

    aux_acc = []
    for z in test_trajectory.acceleration:
        #ax3.scatter(z[0], z[1])
        aux_acc.append(z[0])

    ax3.scatter(test_trajectory.time_split[:-1], aux_acc)




    plt.show()