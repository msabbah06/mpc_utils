import numpy as np

from pathlib import Path
from rosbags.dataframe import get_dataframe
from rosbags.highlevel import AnyReader


def reshape_mpc_data(df_mpc, nx, nv, nb_running_nodes):
    data_mpc = df_mpc.data.to_numpy()
    nb_mpc_iter = data_mpc.shape[0]
    mpc_xs = np.zeros([nb_mpc_iter, nb_running_nodes + 1, nx])
    mpc_us = np.zeros([nb_mpc_iter, nb_running_nodes, nv])
    for mpc_iter in range(nb_mpc_iter):
        for idx in range(nb_running_nodes):
            mpc_xs[mpc_iter, idx, :] = data_mpc[mpc_iter][
                idx * (nx + nv) : idx * (nx + nv) + nx
            ]
            mpc_us[mpc_iter, idx, :] = data_mpc[mpc_iter][
                idx * (nx + nv) + nx : (idx + 1) * (nx + nv)
            ]
        mpc_xs[mpc_iter, nb_running_nodes, :] = data_mpc[mpc_iter][
            (nb_running_nodes - 1) * (nx + nv) : (nb_running_nodes - 1) * (nx + nv) + nx
        ]
    return mpc_xs, mpc_us


def retrieve_mpc_data(bag_path, topic, nx, nv, nb_running_nodes):
    with AnyReader([Path(bag_path)]) as reader:
        df_mpc = get_dataframe(reader, topic, ["data"])
    return reshape_mpc_data(df_mpc, nx, nv, nb_running_nodes)
