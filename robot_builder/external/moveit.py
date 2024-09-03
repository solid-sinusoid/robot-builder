import numpy as np

# from ..elements.joint import Joint
# from ..elements.link import Link
from ..elements.robot import Robot

# from .srdf_xml import SrdfRobot


class MoveitReconfigurator:
    def __init__(self, robot: Robot, update=False):
        self.rb = robot
        self.update = update
        # self.moveit_config_files = {
        #     "ompl": self.rb.robot_package_abs_path + "/config/moveit/ompl_planning.yaml",
        #     "controllers": self.rb.robot_package_abs_path + "/config/moveit/moveit_controllers.yaml",
        #     "joint_limits": self.rb.robot_package_abs_path + "/config/moveit/joint_limits.yaml",
        #     "initial_positions": self.rb.robot_package_abs_path + "/config/moveit/initial_positions.yaml",
        #     "srdf": self.rb.robot_package_abs_path + "/config/moveit/" + self.rb.component_name + ".srdf",
        #     "kinematics": self.rb.robot_package_abs_path + "/config/kinematics.yaml"
        # }

    def _get_adjacency_matrix(self):
        adjacency_matrix = np.zeros(
            (len(self.rb.link_map), len(self.rb.link_map)), dtype=bool
        )
        for joint in self.rb.joints:
            if joint.parent and joint.child:
                parent_link = self.rb.link_map[joint.parent]
                child_link = self.rb.link_map[joint.child]

                if not parent_link.visuals and not child_link.visuals:
                    break

                parent_link_idx = self.rb.links.index(parent_link)
                child_link_idx = self.rb.links.index(child_link)
                adjacency_matrix[parent_link_idx, child_link_idx] = True

        return adjacency_matrix

    def kinematics_yaml(self):
        kinematics = {
            "arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.005,
                "kinematics_solver_attempts": 3,
            }
        }
        # if self.update:
        #     write_yaml_abs(kinematics, self.moveit_config_files["kinematics"])
        return kinematics

    def get_joint_limits(self):
        joint_limits = {}
        # joint_limits["default_velocity_scaling_factor"] = 0.1
        # joint_limits["default_acceleration_scaling_factor"] = 0.1
        joints = {}
        for joint in self.rb.actuated_joint_names:
            joints[joint.name] = {
                "has_velocity_limits": False if joint.max_velocity <= 0 else True,
                "max_velocity": joint.max_velocity,
                "has_acceleration_limits": False,
                "max_acceleration": 0,
            }
        joint_limits["joint_limits"] = joints
        # if self.update:
        #     write_yaml_abs(joint_limits, self.moveit_config_files["joint_limits"])
        return joint_limits

    def get_moveit_controllers(self):
        moveit_controllers = {
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "moveit_simple_controller_manager": {
                "controller_names": ["joint_trajectory_controller"],  # TODO: and this
                "joint_trajectory_controller": {  # TODO: make this changeble
                    "type": "FollowJointTrajectory",
                    "action_ns": "follow_joint_trajectory",
                    "default": True,
                    "joints": self.rb.actuated_joint_names,
                },
            },
        }
        # if self.update:
        #     write_yaml_abs(moveit_controllers, self.moveit_config_files["controllers"])
        return moveit_controllers

    def get_initial_positions(self):
        initial_positions = {}
        joints = {}
        initial_positions["initial_positions"] = self.rb.cfg
        # if self.update:
        #     write_yaml_abs(initial_positions, self.moveit_config_files["initial_positions"])

    def get_srdf(self):
        # plgr = Plan
        # link_names = [link.name for link in self.rb.links]
        # # adjm = self._get_adjacency_matrix(self.rb.robot_joints, self.rb.links)
        # # srdf = Robot(name=self.rb.component_name)
        # plgr = PlanningGroup(
        #     Chain(base_link=link_names[1], tip_link=link_names[-1]),
        #     name="arm"
        # )
        # srdf.extend([plgr])
        # for parent_index, row in enumerate(adjm):
        #     for child_index, value in enumerate(row):
        #         if value == 1:
        #             parent_link = link_names[parent_index]
        #             child_link = link_names[child_index]
        #             srdf.extend([DisableCollision(link1=parent_link, link2=child_link)])
        # return srdf
        pass
