import matplotlib.pyplot as plt


def plot_mpc_iter_durations(title, mpc_iter_durations, time):
    fig, ax = plt.subplots(1, 1)
    fig.canvas.manager.set_window_title(title)
    ax.plot(time, mpc_iter_durations)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("MPC iteration duration (s)")
    plt.show()
