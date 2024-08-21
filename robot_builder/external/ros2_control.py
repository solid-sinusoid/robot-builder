from robot_builder.base import write_yaml_abs

from typing import TYPE_CHECKING

from robot_builder.base import Robot

PARAMETER = "ros__parameters"

class ControllerManager():
    """
    Генерирует готовую конфигурацию для [controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)
    ноды ROS2 для предоставленного объекта робота
    -----------

    Включает в себя следующие контроллеры:
        - joint_trajectory_controller
        - joint_effort_controller
        - cartesian_motion_controller
        - cartesian_force_controller
        - gripper_controller (optional)

    """

    def __init__(self, robot: 'Robot', **kwargs):
        self.robot = robot
        self.parallel_gripper = kwargs.get("parallel_gripper", False)
        self.multifinger_gripper = kwargs.get("multifinger_gripper", False)

    def generate_robot_config(self, robot_name: str | None = None) -> dict:
        """
        Генерирует конфигурацию для всех доступных контроллеров робота на
        основе параметров модели.

        Args:
        ----------
        robot_name: str | None, optional
            Пространство имен робота в ROS2. Если не указано, конфигурация
            будет сгенерирована без учета пространства имен.

        Returns:
        ----------
        dict
            Сформированный конфигурационный файл для ros2_control.
            Если указано `robot_name`, конфигурация будет вложена в словарь
            с этим именем в качестве ключа. В противном случае возвращается
            конфигурация без пространства имен.
        """
        robot_config = {}
        robot_config['controller_manager'] = self.generate_controller_manager()
        robot_config['joint_trajectory_controller'] = self.generate_joint_trajectory_controller()
        robot_config['cartesian_motion_controller'] = self.generate_cartesian_motion_controller()
        robot_config['cartesian_force_controller'] = self.generate_cartesian_force_controller()
        robot_config['joint_effort_controller'] = self.generate_joint_effort_controller()
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
        
        controller_manager_params['joint_trajectory_controller'] = {
            'type': 'joint_trajectory_controller/JointTrajectoryController'}
            
        controller_manager_params['cartesian_motion_controller'] = {
            'type': 'cartesian_motion_controller/CartesianMotionController'}
            
        controller_manager_params['joint_effort_controller'] = {
            'type': 'effort_controllers/JointGroupEffortController'}
            
        if self.parallel_gripper:
            controller_manager_params['gripper_controller'] = {
                'type': 'gripper_controller/GripperController'}

        return {PARAMETER: controller_manager_params}

    def generate_joint_trajectory_controller(self):
        return {
                PARAMETER: {
                'joints': self.robot.actuated_joint_names,
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
                'end_effector_link': self.robot.ee_link,
                'robot_base_link': self.robot.base_link,
                'joints': self.robot.actuated_joint_names,
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

    def generate_cartesian_force_controller(self):
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
                    "link_mass": 0.5
                    },

                "pd_gains": {
                    "trans_x": {'p': 0.05},
                    "trans_y": {'p': 0.05},
                    "trans_z": {'p': 0.05},
                    "rot_x": {'p': 1.5},
                    "rot_y": {'p': 1.5},
                    "rot_z": {'p': 1.5},
                }
        }
    }

    def generate_joint_effort_controller(self):
        return {
                PARAMETER: {
                'joints': self.robot.actuated_joint_names,
            }
        }

    def generate_gripper_controller(self):
        return {
                PARAMETER: {
                'action_monitor_rate': 200.0,
                'joint': self.robot.get_gripper_mimic_joint_name,
                'goal_tolerance': 0.01,
                'max_effort': 5.0,
                'allow_stalling': False,
                'stall_velocity_threshold': 0.001,
                'stall_timeout': 2.0
            }
        }

    @staticmethod
    def save_to_yaml(robot: Robot, package_path: str, filename: str):
        """
        Сохраняет YAML файл для конфигурации контроллеров робота. 
        Создает экземпляр класса `ControllerManager` на основе переданных
        значений генерирует и сохраняет конфигурацию в файл

        Args:
        ----------
        robot: Robot
            Объект робота на основе которого создается конфигурация
        package_path: str
            Путь до пакета в котором необходимо сохранить конфигурацию
        filename: str
            Имя сохраняемого файла
        """
        cm = ControllerManager(
            robot,
            parallel_gripper=robot.parallel_gripper,
            multifinger_gropper=robot.multifinger_gropper)

        robot_config = cm.generate_robot_config(robot.name)
        filepath = f"{package_path}/config/{filename}"
        write_yaml_abs(robot_config, filepath)
