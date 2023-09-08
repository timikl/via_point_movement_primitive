

from class_polynomial import Polynomial
import numpy as np
import matplotlib.pyplot as plt


def combine (array1: np.ndarray, array2: np.ndarray) :
    c = 0
    l = 0
    x = array1.copy ()
    y = array2.copy ()
    for i in x :
        for j in i :
            x[c][l] += y[c][l]
            l += 1
        l = 0
        c += 1
    return x

def doplot (val) :
    test_trajectory = Polynomial(val[0], val[1], val[2], val[3], val[4], val[5], 0.5, 0.01)

    # Create two subplots
    fig, (traj_plot, vel_plot, acc_plot, jerk_plot) = plt.subplots(4, 1)
    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    traj_plot.set_ylabel('Position')
    vel_plot.set_ylabel('Velocity')
    vel_plot.set_xlabel('Time')
    acc_plot.set_ylabel('Acceleration')
    acc_plot.set_xlabel('Time')
    jerk_plot.set_ylabel('Jerk')
    jerk_plot.set_xlabel('Time')

    # print(f"{Fore.LIGHTCYAN_EX}i: {test_trajectory.trajectory} | type: {type(test_trajectory.trajectory)} | len: {len(test_trajectory.trajectory)}")
    # print(f"{Fore.LIGHTMAGENTA_EX}t: {test_trajectory.time_split} | type: {type(test_trajectory.time_split)} | len: {len(test_trajectory.time_split)}")

    # Trajectory plot
    aux_traj = []
    for i in test_trajectory.trajectory:
        # traj_plot.scatter(test_trajectory.time_split,i[1])
        aux_traj.append(i[0])

    traj_plot.scatter(test_trajectory.time_split, aux_traj)

    # Velocity plot
    aux_vel = []
    for j in test_trajectory.velocity:
        # vel_plot.scatter(j[0], j[1])
        aux_vel.append(j[0])

    vel_plot.scatter(test_trajectory.time_split[:], aux_vel)

    # Acceleration plot
    aux_acc = []
    for z in test_trajectory.acceleration:
        # acc_plot.scatter(z[0], z[1])
        aux_acc.append(z[0])

    acc_plot.scatter(test_trajectory.time_split, aux_acc)

    # Jerk plot
    aux_jerk = []

    for k in test_trajectory.jerk:
        # acc_plot.scatter(z[0], z[1])

        aux_jerk.append(k[0])

    # print((len(aux_jerk), len(test_trajectory.time_split)))

    jerk_plot.scatter(test_trajectory.time_split[:-1], aux_jerk)

    plt.show()

if __name__ == "__main__" :
    val1 = np.array ([[0, 0, 0], [0, 0 , 0], [0, 0, 0], [20, 0, 0], [0, 0, 0], [0, 0, 0]])
    print (val1)
    val2 = np.array ([[20, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
    print (val2)
    val3 = combine (val1, val2)
    print (val3)
    doplot (val1)
    doplot (val2)
    doplot (val3)

