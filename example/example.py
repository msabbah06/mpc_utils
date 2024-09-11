import os
from mpc_utils.read_bags_utils import retrieve_mpc_data
import yaml
from mpc_utils.plot_tails import plot_tails
from mpc_utils.robot import Robot

robot = Robot('models/others/robots/human_urdf/urdf/robot.urdf','models/others/robots')
model = robot.model

with open("example/mpc_config.yaml", "r") as file:
    mpc_config = yaml.safe_load(file)

bag_path = os.path.join(mpc_config["bag_directory"], mpc_config["bag_name"])
if "mpc_data_topic_name" in mpc_config.keys():
    mpc_xs, mpc_us = retrieve_mpc_data(
        bag_path,
        mpc_config["mpc_data_topic_name"],
        24,
        7,
        mpc_config["nb_running_nodes"],
    )

print(mpc_xs.shape, mpc_us.shape)

plot_tails(mpc_xs, mpc_us, robot.model, mpc_config)
