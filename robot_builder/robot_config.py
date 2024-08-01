from typing import List, Optional

from ament_index_python.packages import get_package_share_directory

# from robot_builder.components import Component, JointNode, LinkNode
from robot_builder.moveit import MoveitReconfigurator
from robot_builder.ros2_control import ControllerManager
from robot_builder.utils import *

from robot_builder.urdf import URDF, Robot, Link, Joint, Ros2Control

HARDWARE = {"gazebo": "ign_ros2_control/IgnitionSystem"}

class RobotConfig:
    def __init__(self, robot_name: str, robot_type: str, parent: str) -> None:
        # super().__init__(robot_name)
        self.parts: List[str] = []
        self.urdf = None
        self.robot = Robot(robot_name)

    def add_part(self, part: str) -> None:
        self.parts.append(part)

    def add_link(self, link: Link):
        self.robot.links.append(link)

    def add_joint(self, joint: Joint):
        self.robot.joints.append(joint)

    def add_interface(self, hw: Ros2Control) -> None:
        self.robot.control.append(hw)

    def generate_moveit_config(self, update=False):
        moveit_config = MoveitReconfigurator(self, update)
        kinematics = moveit_config.kinematics_yaml()
        joint_limits = moveit_config.get_joint_limits()
        controllers = moveit_config.get_moveit_controllers()
        initial_positions = moveit_config.get_initial_positions()

    def add_gripper(self) -> None:
        pass

    def add_controller_manager(self, robot_name: str, config: dict) -> None:
        cm = ControllerManager(self, config)

    def add_group(self) -> None:
        pass

    def add_simulation(self, simulator: str) -> None:
        pass

    def list_parts(self) -> None:
        print(f"Robot config parts: {', '.join(self.parts)}\n", end="")

    def srdf(self) -> Robot:
        return MoveitReconfigurator(self, False).get_srdf()
