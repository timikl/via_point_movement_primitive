import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from colorama import Fore
from class_polynomial import Polynomial

if __name__ == "__main__":
    test_trajectory = Polynomial([0, 0, 0], [0, 10, 0], [0, 0, 0], [10, 10, 10], [0, 0, 0], [0, 0, 0], 0.5, 0.01)

    # Create time array
    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    # Print trajectory, velocity, acceleration, and jerk values
    for index, i in enumerate(time):
        print(
            f"{Fore.BLUE}||{index}|| {Fore.LIGHTBLACK_EX} Pose: {test_trajectory.trajectory[index]} | {Fore.GREEN}Vel: {test_trajectory.velocity[index]} m/s | {Fore.RED}Acc: {test_trajectory.acceleration[index]} | {Fore.CYAN}Jerk: {test_trajectory.jerk[index-1]}")

    # Create subplots for trajectory, velocity, acceleration, and jerk
    fig = plt.figure(figsize=(10, 10))

    ax1 = fig.add_subplot(221, projection="3d")
    ax1.scatter3D(*zip(*test_trajectory.trajectory), c='b')
    ax1.set_title("Trajectory")

    ax2 = fig.add_subplot(222, projection="3d")
    ax2.scatter3D(*zip(*test_trajectory.velocity), c='g')
    ax2.set_title("Velocity")

    ax3 = fig.add_subplot(223, projection="3d")
    ax3.scatter3D(*zip(*test_trajectory.acceleration), c='r')
    ax3.set_title("Acceleration")

    ax4 = fig.add_subplot(224, projection="3d")
    ax4.scatter3D(*zip(*test_trajectory.jerk), c='c')
    ax4.set_title("Jerk")

    plt.tight_layout()
    plt.show()