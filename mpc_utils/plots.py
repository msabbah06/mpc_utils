import matplotlib.pyplot as plt


def plot_mpc_iter_durations(title, mpc_iter_durations, time):
    fig, ax = plt.subplots(1, 1)
    fig.canvas.manager.set_window_title(title)
    ax.plot(time, mpc_iter_durations)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("MPC iteration duration (s)")
    plt.show()


def plot_xyz_traj(title, translation_pose_data, time):
    fig, ax = plt.subplots(3, 1)
    fig.canvas.manager.set_window_title(title)
    axes = ["x", "y", "z"]
    for i in range(3):
        ax[i].plot(time, translation_pose_data[:, i])
        ax[i].set_xlabel("t (s)")
        ax[i].set_ylabel(axes[i])
    plt.show()
