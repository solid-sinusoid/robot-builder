from typing import Optional
from robot_builder.utils import write_yaml_abs

PARAMETER = "ros__parameters"

class ControllerManager():
    def __init__(self, robot_base, config: dict):
        self.rb = robot_base
        self.config = config
        self.non_fixed_joints = [joint.name for joint in robot_base.this_robot_joints if joint.joint_type != "fixed"]
        if robot_base.children:
            self.non_fixed_joints.append(robot_base.children[0].robot_joints[1].name) #FIXME: this is hardcode
        self.all_joints = [joint.name for joint in robot_base.joints]
        self.links = [link.name for link in robot_base.links]
        self.is_gripper = self.config.get("gripper_controller", False)

    def generate_robot_config(self, robot_name: Optional[str] = None):
        robot_config = {}
        robot_config['controller_manager'] = self.generate_controller_manager()
        if self.config.get('joint_trajectory_controller', False):
            robot_config['joint_trajectory_controller'] = self.generate_joint_trajectory_controller()
        if self.config.get('cartesian_motion_controller', False):
            robot_config['cartesian_motion_controller'] = self.generate_cartesian_motion_controller()
        if self.config.get('joint_effort_controller', False):
            robot_config['joint_effort_controller'] = self.generate_joint_effort_controller()
        if self.config.get('gripper_controller', False):
            robot_config['gripper_controller'] = self.generate_gripper_controller()
        if robot_name is None:
            return robot_config
        else:
            return {robot_name: robot_config}

    def generate_controller_manager(self):
        controller_manager_params = {}
        controller_manager_params['update_rate'] = 1000
        controller_manager_params['joint_state_broadcaster'] = {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'}
        
        if self.config.get('joint_trajectory_controller', False):
            controller_manager_params['joint_trajectory_controller'] = {
                'type': 'joint_trajectory_controller/JointTrajectoryController'}
            
        if self.config.get('cartesian_motion_controller', False):
            controller_manager_params['cartesian_motion_controller'] = {
                'type': 'cartesian_motion_controller/CartesianMotionController'}
            
        if self.config.get('joint_effort_controller', False):
            controller_manager_params['joint_effort_controller'] = {
                'type': 'effort_controllers/JointGroupEffortController'}
            
        if self.config.get('gripper_controller', False):
            controller_manager_params['gripper_controller'] = {
                'type': 'gripper_controller/GripperController'}

        return {PARAMETER: controller_manager_params}

    def generate_joint_trajectory_controller(self):
        return {
                PARAMETER: {
                'joints': self.non_fixed_joints,
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
                'end_effector_link': self.links[-1],
                'robot_base_link': self.links[0],
                'joints': self.non_fixed_joints,
                'command_interfaces': ['position'],
                'solver': {
                    'error_scale': 1.0,
                    'iterations': 10,
                    'publish_state_feedback': True
                },
                'pd_gains': {
                    'trans_x': {'p': 1.0},
                    'trans_y': {'p': 1.0},
                    'trans_z': {'p': 1.0},
                    'rot_x': {'p': 0.5},
                    'rot_y': {'p': 0.5},
                    'rot_z': {'p': 0.5}
                }
            }
        }

    def generate_joint_effort_controller(self):
        return {
                PARAMETER: {
                'joints': self.non_fixed_joints,
            }
        }

    def generate_gripper_controller(self):
        if self.rb.children:
            joint = self.rb.children[-1].joints[-1].name
        else:
            joint = self.all_joints[-1]
        return {
                PARAMETER: {
                'action_monitor_rate': 200.0,
                'joint': joint,
                'goal_tolerance': 0.01,
                'max_effort': 5.0,
                'allow_stalling': False,
                'stall_velocity_threshold': 0.001,
                'stall_timeout': 2.0
            }
        }

    def save_to_yaml(self, filename, robot_name):
        robot_config = self.generate_robot_config() #TODO: add robot name for namespace
        filepath = f"{self.rb.robot_package_abs_path}/config/{filename}"
        write_yaml_abs(robot_config, filepath)
