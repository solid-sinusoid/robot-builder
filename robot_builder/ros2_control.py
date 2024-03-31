from typing import Optional
from utils import write_yaml_abs

PARAMETER = "ros__parameters"

class ControllerManager():
    def __init__(self, robot_base):
        self.rb = robot_base
        self.config = {
            'joint_trajectory_controller': True,
            'cartesian_motion_controller': True,
            'joint_effort_controller': True,
            'gripper_controller': False}

    def generate_robot_config(self, robot_name: Optional[str]):
        robot_config = {}
        robot_config['controller_manager'] = self.generate_controller_manager()
        if self.config['joint_trajectory_controller']:
            robot_config['joint_trajectory_controller'] = self.generate_joint_trajectory_controller()
        if self.config['cartesian_motion_controller']:
            robot_config['cartesian_motion_controller'] = self.generate_cartesian_motion_controller()
        if self.config['joint_effort_controller']:
            robot_config['joint_effort_controller'] = self.generate_joint_effort_controller()
        if self.config['gripper_controller']:
            robot_config['gripper_controller'] = self.generate_gripper_controller()
        if robot_name is None:
            return robot_config
        else:
            return {robot_name: robot_config}

    def generate_controller_manager(self):
        controller_manager_params = {}
        controller_manager_params['update_rate'] = 1000
        controller_manager_params['joint_state_broadcaster'] = {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
        if self.config['joint_trajectory_controller']:
            controller_manager_params['joint_trajectory_controller'] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
        if self.config['cartesian_motion_controller']:
            controller_manager_params['cartesian_motion_controller'] = {'type': 'cartesian_motion_controller/CartesianMotionController'}
        if self.config['joint_effort_controller']:
            controller_manager_params['joint_effort_controller'] = {'type': 'effort_controllers/JointGroupEffortController'}
        return {PARAMETER: controller_manager_params}

    def generate_joint_trajectory_controller(self):
        return {
                PARAMETER: {
                'joints': self.rb.joints,
                'command_interfaces': ['position'],
                'state_interfaces': ['position', 'velocity'],
                'state_publish_rate': 100.0,
                'action_monitor_rate': 20.0,
                'allow_partial_joints_goal': False
            }
        }

    def generate_cartesian_motion_controller(self):
        return {
                PARAMETER: {
                'end_effector_link': self.rb.joints[-1],
                'robot_base_link': self.rb.joints[0],
                'joints': self.rb.joints,
                'command_interfaces': ['position'],
                'solver': {
                    'error_scale': 1.0,
                    'iterations': 10,
                    'publish_state_feedback': True
                },
                'pd_gains': {
                    'trans_x': {'p': 100.0},
                    'trans_y': {'p': 100.0},
                    'trans_z': {'p': 100.0},
                    'rot_x': {'p': 50.5},
                    'rot_y': {'p': 50.5},
                    'rot_z': {'p': 50.5}
                }
            }
        }

    def generate_joint_effort_controller(self):
        return {
                PARAMETER: {
                'joints': self.rb.joints,
            }
        }

    def generate_gripper_controller(self):
        return {
                PARAMETER: {
                'action_monitor_rate': 200.0,
                'joint': 'rbs_gripper_r_finger_joint',
                'goal_tolerance': 0.01,
                'max_effort': 5.0,
                'allow_stalling': False,
                'stall_velocity_threshold': 0.001,
                'stall_timeout': 2.0
            }
        }

    def save_to_yaml(self, filename, robot_name):
        robot_config = self.generate_robot_config(robot_name)
        filepath = f"{self.rb.robot_package_abs_path}/config/{filename}"
        write_yaml_abs(robot_config, filepath)
