import numpy as np
from .components import JointNode, LinkNode
from .utils import *
from typing import List

class MoveitReconfigurator:
    def __init__(self, robot_base, update=False):
        self.rb = robot_base
        self.update = update
        self.non_fixed_joints = [joint.name for joint in robot_base.this_robot_joints if joint.joint_type != "fixed"]
        if robot_base.children:
            self.non_fixed_joints.append(robot_base.children[0].robot_joints[1].name) #FIXME: this is hardcode
        self.moveit_config_files = {
            "ompl": self.rb.robot_package_abs_path + "/config/moveit/ompl_planning.yaml",
            "controllers": self.rb.robot_package_abs_path + "/config/moveit/moveit_controllers.yaml",
            "joint_limits": self.rb.robot_package_abs_path + "/config/moveit/joint_limits.yaml",
            "initial_positions": self.rb.robot_package_abs_path + "/config/moveit/initial_positions.yaml",
            "srdf": self.rb.robot_package_abs_path + "/config/moveit/" + self.rb.component_name + ".srdf",
            "kinematics": self.rb.robot_package_abs_path + "/config/kinematics.yaml"
        }

    def _get_adjacency_matrix(self, joints: List[JointNode], links: List[LinkNode]):
        node_names = [node.name for node in links]
        adjacency_matrix = np.zeros((len(node_names), len(node_names)), dtype=int)
        link_fixed = []
        link_fixed_parent = []

        for joint in joints:
            if joint.joint_type == "fixed":
                link_fixed.append(joint.child)
                link_fixed_parent.append(joint.parent)
                if joint.parent in link_fixed:
                    parent_index = node_names.index(link_fixed_parent[0])
                    child_index = node_names.index(joint.child)
                    adjacency_matrix[parent_index, child_index] = 1
            else:
                parent_index = node_names.index(joint.parent)
                child_index = node_names.index(joint.child)
                adjacency_matrix[parent_index, child_index] = 1

        return adjacency_matrix

    def kinematics_yaml(self):
        kinematics = {
                        "arm": {
                            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                            "kinematics_solver_search_resolution": 0.005,
                            "kinematics_solver_timeout": 0.005,
                            "kinematics_solver_attempts": 3
                        }
                    }
        if self.update:
            write_yaml_abs(kinematics, self.moveit_config_files["kinematics"])
        return kinematics

    def get_joint_limits(self):
        joint_limits = {}
        # joint_limits["default_velocity_scaling_factor"] = 0.1
        # joint_limits["default_acceleration_scaling_factor"] = 0.1
        joints = {}
        for joint in self.rb.this_robot_joints:
            joints[joint.name] = {
                "has_velocity_limits": False if joint.max_velocity <= 0 else True,
                "max_velocity": joint.max_velocity,
                "has_acceleration_limits": False,
                "max_acceleration": 0
            }
        joint_limits["joint_limits"] = joints
        if self.update:
            write_yaml_abs(joint_limits, self.moveit_config_files["joint_limits"])
        return joint_limits

    def get_moveit_controllers(self):
        moveit_controllers = {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": {
                "controller_names": ["joint_trajectory_controller"], #TODO: and this
                "joint_trajectory_controller": { #TODO: make this changeble
                    "type": "FollowJointTrajectory",
                    "action_ns": "follow_joint_trajectory",
                    "default": True,
                    "joints": self.non_fixed_joints
                }
            },
        }
        if self.update:
            write_yaml_abs(moveit_controllers, self.moveit_config_files["controllers"])
        return moveit_controllers

    def get_initial_positions(self):
        initial_positions = {}
        joints = {}
        for joint in self.rb.this_robot_joints:
            joints[joint.name] = 0
        initial_positions["initial_positions"] = joints
        if self.update:
            write_yaml_abs(initial_positions, self.moveit_config_files["initial_positions"])

    def get_srdf(self):
        link_names = [link.name for link in self.rb.links]
        adjm = self._get_adjacency_matrix(self.rb.robot_joints, self.rb.links)
        srdf = Robot(name=self.rb.component_name)
        plgr = PlanningGroup(
            Chain(base_link=link_names[1], tip_link=link_names[-1]),
            name="arm"
        )
        srdf.extend([plgr])
        for parent_index, row in enumerate(adjm):
            for child_index, value in enumerate(row):
                if value == 1:
                    parent_link = link_names[parent_index]
                    child_link = link_names[child_index]
                    srdf.extend([DisableCollision(link1=parent_link, link2=child_link)])
        return srdf
