import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from colorama import Fore
from class_polynomial import Polynomial

if __name__ == "__main__":

    test_trajectory = Polynomial([0, 0, 0], [0, 60, 0], [0, 0, 0], [10, 10, 10], [0, 0, 0], [0, 0, 0], 0.5, 0.01)

    # Create two subplots
    fig = plt.figure()
    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    for index, i in enumerate(time):
        # prints velocity and acceleration
        print(
            f"{Fore.BLUE}||{index}|| {Fore.LIGHTBLACK_EX} Pose: {test_trajectory.trajectory[index]}|{Fore.GREEN}Vel : {test_trajectory.velocity[index]} m/s | {Fore.RED} Acc: {test_trajectory.acceleration[index]} | {Fore.CYAN} Jerk: {test_trajectory.jerk[index-1]}")



    ax = fig.add_subplot(111, projection="3d")

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(8, 12), subplot_kw={'projection': '3d'})



    def plot_variables(what_to_plot):
        for k in what_to_plot:
            ax1.scatter3D(k[0], k[1], k[2])
        plt.show()

    plot_variables(test_trajectory.trajectory)
    #plot_variables(test_trajectory.velocity)
    #plot_variables(test_trajectory.acceleration)
    #plot_variables(test_trajectory.jerk)
    
    