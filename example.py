import os

import example_robot_data
import pinocchio as pin
from read_bags_utils import get_mpc_xs_us
import yaml
from plot_tails import plot_tails


def get_robot_model(robot):
    locked_joints = [
        robot.model.getJointId("panda_finger_joint1"),
        robot.model.getJointId("panda_finger_joint2"),
    ]

    urdf_path = "robot.urdf"
    srdf_path = "demo.srdf"

    model = pin.Model()
    pin.buildModelFromUrdf(urdf_path, model)
    pin.loadReferenceConfigurations(model, srdf_path, False)
    q0 = model.referenceConfigurations["default"]
    return pin.buildReducedModel(model, locked_joints, q0)


robot = example_robot_data.load("panda")
model = get_robot_model(robot)
robot.model = model
with open("mpc_config.yaml", "r") as file:
    mpc_config = yaml.safe_load(file)
bag_path = os.path.join(mpc_config["bag_directory"], mpc_config["bag_name"])
if "mpc_data_topic_name" in mpc_config.keys():
    mpc_xs, mpc_us = get_mpc_xs_us(
        bag_path,
        mpc_config["mpc_data_topic_name"],
        robot.model.nq + robot.model.nv,
        robot.model.nv,
        mpc_config["nb_running_nodes"],
    )

plot_tails(mpc_xs, mpc_us, robot.model, mpc_config)
