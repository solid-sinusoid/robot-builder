from ..elements.robot import Robot
from ..utils import write_yaml_abs

PARAMETER = "ros__parameters"


class ControllerManager:
    """
    Generates a complete configuration for the [controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)
    node in ROS2 for the provided robot object.

    Features the following controllers:
        - joint_trajectory_controller
        - joint_effort_controller
        - cartesian_motion_controller
        - cartesian_force_controller
        - gripper_controller (optional)
    """

    def __init__(self, robot: "Robot", **kwargs):
        """
        Initializes the ControllerManager instance.

        Args:
        ----------
        robot: Robot
            The robot object for which the configuration will be generated.

        **kwargs: Additional keyword arguments

            parallel_gripper: bool (optional)
                Indicates whether to use a parallel gripper.

            multifinger_gripper: bool (optional)
                Indicates whether to use a multifinger gripper.
        """
        self.robot = robot
        self.parallel_gripper = kwargs.get("parallel_gripper", False)
        self.multifinger_gripper = kwargs.get("multifinger_gripper", False)

    def generate_robot_config(self, robot_name: str | None = None) -> dict:
        """
        Generates the configuration for all available robot controllers based on model parameters.

        Args:
        ----------
        robot_name: str | None, optional
            The robot's namespace in ROS2. If not provided, the configuration
            will be generated without a namespace.

        Returns:
        ----------
        dict
            The generated configuration file for ros2_control.
            If `robot_name` is provided, the configuration is nested within a dictionary
            with this name as the key. Otherwise, the configuration is returned
            without a namespace.
        """
        robot_config = {
            "controller_manager": self.generate_controller_manager(),
            "joint_trajectory_controller": self.generate_joint_trajectory_controller(),
            "cartesian_motion_controller": self.generate_cartesian_motion_controller(),
            "cartesian_force_controller": self.generate_cartesian_force_controller(),
            "joint_effort_controller": self.generate_joint_effort_controller(),
            "motion_control_handle": self.generate_motion_controller_handle(),
            "force_torque_sensor_broadcaster": self.generate_force_torque_sensor_broadcaster(),
            "gripper_controller": self.generate_gripper_controller(),
        }
        return {robot_name: robot_config} if robot_name else robot_config

    def generate_motion_controller_handle(self):
        return {
            PARAMETER: {
                "end_effector_link": self.robot.ee_link,
                "robot_base_link": self.robot.base_link,
                "ft_sensor_ref_link": "tool0",
                "controller_name": "cartesian_motion_controller",
                "joints": self.robot.actuated_joint_names,
            }
        }

    def generate_force_torque_sensor_broadcaster(self):
        return {
            PARAMETER: {
                "frame_id": "tool0",
                "sensor_name": "fts_sensor"
            }
        }

    def generate_controller_manager(self):
        """
        Generates the controller manager parameters.

        Returns:
        ----------
        dict
            A dictionary containing the parameters for the controller manager.
        """
        controller_manager_params = {
            "update_rate": 1000,
            "joint_state_broadcaster": {
                "type": "joint_state_broadcaster/JointStateBroadcaster"
            },
            "joint_trajectory_controller": {
                "type": "joint_trajectory_controller/JointTrajectoryController"
            },
            "cartesian_motion_controller": {
                "type": "cartesian_motion_controller/CartesianMotionController"
            },
            "joint_effort_controller": {
                "type": "effort_controllers/JointGroupEffortController"
            },
            "motion_control_handle": {
                "type": "cartesian_controller_handles/MotionControlHandle"
            }
        }

        if self.parallel_gripper:
            controller_manager_params["gripper_controller"] = {
                "type": "position_controllers/GripperActionController"
            }

        return {PARAMETER: controller_manager_params}

    def generate_joint_trajectory_controller(self):
        """
        Generates the configuration for the joint trajectory controller.

        Returns:
        ----------
        dict
            A dictionary containing the configuration for the joint trajectory controller.
        """
        sin = {
            name
            for cont in self.robot.control
            for ji in cont.joint_interface
            if ji.name in self.robot.actuated_joint_names
            for name in ji.get_list_of_state_interface_names
        }

        cin = {
            name
            for cont in self.robot.control
            for ji in cont.joint_interface
            if ji.name in self.robot.actuated_joint_names
            for name in ji.get_list_of_command_interface_names
        }

        sin_list = list(sin)
        cin_list = list(cin)

        return {
            PARAMETER: {
                "joints": self.robot.actuated_joint_names,
                "command_interfaces": cin_list,
                "state_interfaces": sin_list,
                "state_publish_rate": 100.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": False,
            }
        }

    def generate_cartesian_motion_controller(self):
        """
        Generates the configuration for the cartesian motion controller.

        Returns:
        ----------
        dict
            A dictionary containing the configuration for the cartesian motion controller.
        """
        return {
            PARAMETER: {
                "end_effector_link": self.robot.ee_link,
                "robot_base_link": self.robot.base_link,
                "joints": self.robot.actuated_joint_names,
                "command_interfaces": ["position"],
                "solver": {
                    "error_scale": 1.0,
                    "iterations": 10,
                    "publish_state_feedback": True,
                },
                "pd_gains": {
                    "trans_x": {"p": 1.0},
                    "trans_y": {"p": 1.0},
                    "trans_z": {"p": 1.0},
                    "rot_x": {"p": 0.5},
                    "rot_y": {"p": 0.5},
                    "rot_z": {"p": 0.5},
                },
            }
        }

    def generate_cartesian_force_controller(self):
        """
        Generates the configuration for the cartesian force controller.

        Returns:
        ----------
        dict
            A dictionary containing the configuration for the cartesian force controller.
        """
        return {
            PARAMETER: {
                "end_effector_link": self.robot.ee_link,
                "robot_base_link": self.robot.base_link,
                "ft_sensor_ref_link": "tool0",
                "ft_sensor_node_name": "force_torque_sensor_broadcaster",
                "joints": self.robot.actuated_joint_names,
                "command_interfaces": ["velocity"],
                "solver": {
                    "error_scale": 0.5,
                    "publish_state_feedback": True,
                    "link_mass": 0.5,
                },
                "pd_gains": {
                    "trans_x": {"p": 0.05},
                    "trans_y": {"p": 0.05},
                    "trans_z": {"p": 0.05},
                    "rot_x": {"p": 1.5},
                    "rot_y": {"p": 1.5},
                    "rot_z": {"p": 1.5},
                },
            }
        }

    def generate_joint_effort_controller(self):
        """
        Generates the configuration for the joint effort controller.

        Returns:
        ----------
        dict
            A dictionary containing the configuration for the joint effort controller.
        """
        return {
            PARAMETER: {
                "joints": self.robot.actuated_joint_names,
            }
        }

    def generate_gripper_controller(self):
        """
        Generates the configuration for the gripper controller.

        Returns:
        ----------
        dict
            A dictionary containing the configuration for the gripper controller.
        """
        return {
            PARAMETER: {
                "action_monitor_rate": 200.0,
                "joint": self.robot.gripper_actuated_joint_names[0],
                "goal_tolerance": 0.01,
                "max_effort": 5.0,
                "allow_stalling": False,
                "stall_velocity_threshold": 0.001,
                "stall_timeout": 2.0,
            }
        }

    @staticmethod
    def save_to_yaml(robot: Robot, package_path: str, filename: str, general: bool = True):
        """
        Saves a YAML file for the robot controller configuration.
        Creates an instance of the `ControllerManager` class based on the provided
        values and generates and saves the configuration to a file.

        Args:
        ----------
        robot: Robot
            The robot object used to create the configuration.
        package_path: str
            The path to the package where the configuration should be saved.
        filename: str
            The name of the file to save the configuration.
        general: bool = [True]
            If true then controller manager configuration will be without namespace
        """
        cm = ControllerManager(
            robot,
            parallel_gripper=robot.parallel_gripper,
        )

        if not general:
            robot_config = cm.generate_robot_config(robot.name)
        else:
            robot_config = cm.generate_robot_config()

        filepath = f"{package_path}/config/{filename}"
        write_yaml_abs(robot_config, filepath)
