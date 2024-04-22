import numpy as np
from robot_builder.components import JointNode, LinkNode
from robot_builder.utils import *
from typing import List

class MoveitReconfigurator:
    def __init__(self, robot_base, update=False):
        self.rb = robot_base
        self.update = update
        self.moveit_config_files = {
            "ompl": self.rb.robot_package_abs_path + "/config/moveit/ompl_planning.yaml",
            "controllers": self.rb.robot_package_abs_path + "/config/moveit/moveit_controllers.yaml",
            "joint_limits": self.rb.robot_package_abs_path + "/config/moveit/joint_limits.yaml",
            "initial_positons": self.rb.robot_package_abs_path + "/config/moveit/initial_positons.yaml",
            "srdf": self.rb.robot_package_abs_path + "/config/moveit/" + self.rb.robot_type + ".srdf"
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

    def get_joint_limits(self):
        joint_limits = load_yaml_abs(self.moveit_config_files["joint_limits"])
        if 'joint_limits' in joint_limits:
            old_joint_names = list(joint_limits["joint_limits"].keys())
            new_joint_names = set(self.rb.joints) - set(old_joint_names)
            
            for old_joint, new_joint in zip(old_joint_names, self.rb.joints):
                joint_limits["joint_limits"][new_joint] = joint_limits["joint_limits"].pop(old_joint)
            
            if len(new_joint_names) > 0:
                self._build_joint_limits_config(joint_limits, new_joint_names)
            elif len(new_joint_names) < 0:
                self._remove_extra_joints()
            if self.update:
                write_yaml_abs(joint_limits, self.moveit_config_files["joint_limits"])
        return joint_limits

    def _get_joint_group(self, joint:str):
        for key, value_list in self.rb.group_link_joint.items():
            if joint in value_list:
                return key

    def _remove_extra_joints(self):
        pass

    def _build_joint_limits_config(self, joint_limits, joint_names):
        for joint_name in joint_names:
            joint_group = self._get_joint_group(joint_name)
            if joint_group:
                has_vel_limit = True if self.rb.robot_config.get(joint_group, {}).get('joint', {}).get('max_velocity') is not None else False
                max_vel = self.rb.robot_config.get(joint_group, {}).get('joint', {}).get('max_velocity') if has_vel_limit else 0
                config = {
                    "has_velocity_limits": has_vel_limit,
                    "max_velocity": max_vel,
                    "has_acceleration_limits": False,
                    "max_acceleration": 0,
                }
                joint_limits["joint_limits"][joint_name] = config

    def get_moveit_controllers(self):
        moveit_controllers = load_yaml_abs(self.moveit_config_files["controllers"])
        if 'moveit_simple_controller_manager' in moveit_controllers:
            controller_manager = moveit_controllers['moveit_simple_controller_manager']
            if 'joint_trajectory_controller' in controller_manager:
                controller = controller_manager['joint_trajectory_controller']
                if 'joints' in controller:
                    controller['joints'] = self.rb.joints
        if self.update:
            write_yaml_abs(moveit_controllers, self.moveit_config_files["controllers"])
        return moveit_controllers

    def get_srdf(self):
        link_names = [link.name for link in self.rb.links]
        adjm = self._get_adjacency_matrix(self.rb.joints, self.rb.links)
        srdf = Robot(name=self.rb.robot_type)
        plgr = PlanningGroup(
            Chain(base_link=link_names[0], tip_link=link_names[-1]),
            name=self.rb.robot_type
        )
        srdf.extend([plgr])
        for parent_index, row in enumerate(adjm):
            for child_index, value in enumerate(row):
                if value == 1:
                    parent_link = link_names[parent_index]
                    child_link = link_names[child_index]
                    srdf.extend([DisableCollision(link1=parent_link, link2=child_link)])

        return srdf
