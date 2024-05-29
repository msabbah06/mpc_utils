# MPC_utils
This project is based on the work of this repository : https://github.com/machines-in-motion/minimal_examples_crocoddyl to plot tails of your mpc.

# Usage
To use this project, you should edit mpc_config.yaml with your own parameters, and call the plot_tails function, the arguments of the function are :
- mpc_xs : numpy array storing the prediction xs at each iteration of the mpc
    shape : np.ndarray[number of iteration of your mpc, number of nodes, size of state vector]
- mpc_us : numpy array storing the prediction us at each iteration of the mpc
    shape : np.ndarray[number of iteration of your mpc, number of nodes-1, size of control vector]
- model : pinocchio model of the robot
- mpc_config : dictionary loaded from the mpc_config.yaml file

Optional parameters:
- ctrl_refs : control reference of your first node at each mpc iteration,
    shape : np.ndarray[number of iteration of your mpc, size of control vector]
- state_refs : state reference of your first node at each mpc iteration,
    shape : np.ndarray[number of iteration of your mpc, size of state vector]
- translation_refs : translation reference control of your first node at each mpc iteration,
    shape : np.ndarray[number of iteration of your mpc, 3]