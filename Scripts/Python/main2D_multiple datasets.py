import matplotlib.pyplot as plt
import numpy as np
from class_polynomial import Polynomial

def plot_data(axs, params, label_prefix):
    test_trajectory = Polynomial(*params)

    test_trajectory.whole_trajectory_calculate()
    test_trajectory.velocity_calculate()
    test_trajectory.acceleration_calculate()
    test_trajectory.jerk_calculate()

    # Trajectory plot
    aux_traj = np.array([i[0] for i in test_trajectory.trajectory])
    axs[0].plot(test_trajectory.time_split, aux_traj, label=f'{label_prefix} Position')

    # Velocity plot
    aux_vel = np.array([j[0] for j in test_trajectory.velocity])
    axs[1].plot(test_trajectory.time_split, aux_vel, label=f'{label_prefix} Velocity')

    # Acceleration plot
    aux_acc = np.array([z[0] for z in test_trajectory.acceleration])
    axs[2].plot(test_trajectory.time_split[1:-1], aux_acc[1:-1], label=f'{label_prefix} Acceleration')

    # Jerk plot
    aux_jerk = np.array([k[0] for k in test_trajectory.jerk])
    axs[3].plot(test_trajectory.time_split[2:-1], aux_jerk[2:], label=f'{label_prefix} Jerk')

    # Return the computed values as matrices and the time_split for plotting
    return {
        'trajectory': aux_traj.reshape(1, -1),
        'velocity': aux_vel.reshape(1, -1),
        'acceleration': aux_acc.reshape(1, -1),
        'jerk': aux_jerk.reshape(1, -1),
        'time_split': test_trajectory.time_split
    }

if __name__ == "__main__":
    fig, axs = plt.subplots(4, 1, figsize=(10, 15))

    base_params = ([0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0.5, 0.01)
    target_positions = [[i, i, i] for i in range(10, 101, 10)]

    all_data = {
        'trajectory': [],
        'velocity': [],
        'acceleration': [],
        'jerk': []
    }

    for index, target in enumerate(target_positions):
        params = list(base_params)
        params[3] = target
        data = plot_data(axs, params, f'Set {index + 1}')
        for key in all_data:
            all_data[key].append(data[key])

    # Convert lists of matrices to single matrices
    for key in all_data:
        all_data[key] = np.vstack(all_data[key])

    # Compute the mean for each category and plot it
    mean_data = {}
    time_split = data['time_split']  # Use the time_split from the last iteration
    for key in all_data:
        mean_data[key] = np.mean(all_data[key], axis=0)
        if key == 'trajectory':
            axs[0].plot(time_split, mean_data[key], label='Mean Position', linestyle='--', color='black')
        elif key == 'velocity':
            axs[1].plot(time_split, mean_data[key], label='Mean Velocity', linestyle='--', color='black')
        elif key == 'acceleration':
            axs[2].plot(time_split[1:-1], mean_data[key][1:-1], label='Mean Acceleration', linestyle='--', color='black')
        elif key == 'jerk':
            axs[3].plot(time_split[2:-1], mean_data[key][2:], label='Mean Jerk', linestyle='--', color='black') #I have to crop in the begining and the end, because of derived values -> 0

    for ax in axs:
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.show()

    # Print or process the stored data as needed
    #print(all_data)
    print(mean_data)
