from class_polynomial import Polynomial
import numpy as np
import matplotlib.pyplot as plt


def combine(array1: np.ndarray, array2: np.ndarray) -> np.ndarray:
    """Combine two numpy arrays by element-wise addition."""
    return array1 + array2


def doplot(axs, val, label_prefix):
    test_trajectory = Polynomial(val[0], val[1], val[2], val[3], val[4], val[5], val[6] 0.5, 0.01)

    time = np.linspace(0, test_trajectory.t, int(test_trajectory.t / test_trajectory.dt))

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    # Trajectory plot
    aux_traj = [i[0] for i in test_trajectory.trajectory]
    axs[0].plot(test_trajectory.time_split, aux_traj, label=f'{label_prefix} Position')

    # Velocity plot
    aux_vel = [j[0] for j in test_trajectory.velocity]
    axs[1].plot(test_trajectory.time_split, aux_vel, label=f'{label_prefix} Velocity')

    # Acceleration plot
    aux_acc = [z[0] for z in test_trajectory.acceleration]
    axs[2].plot(test_trajectory.time_split, aux_acc, label=f'{label_prefix} Acceleration')

    # Jerk plot
    aux_jerk = [k[0] for k in test_trajectory.jerk]
    axs[3].plot(test_trajectory.time_split[:-1], aux_jerk, label=f'{label_prefix} Jerk')


if __name__ == "__main__":
    fig, axs = plt.subplots(4, 1, figsize=(10, 15))  # Create 4 subplots in a single column

    # x0, dx0, ddx0, x1, dx1, ddx1, t, dt

    val1 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
    val2 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0]])
    val3 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [2, 0, 0], [0, 0, 0], [0, 0, 0]])
    val4 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [3, 0, 0], [0, 0, 0], [0, 0, 0]])
    val5 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [4, 0, 0], [0, 0, 0], [0, 0, 0]])
    val6 = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [5, 0, 0], [0, 0, 0], [0, 0, 0]])

    doplot(axs, val1, 'Val1')
    doplot(axs, val2, 'Val2')
    doplot(axs, val3, 'Val3')
    doplot(axs, val4, 'Val4')
    doplot(axs, val5, 'Val5')


    for ax in axs:
        ax.legend()  # Add legend to differentiate the plots

    plt.tight_layout()
    plt.show()
