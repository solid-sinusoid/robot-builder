import enum
from os import name
from typing import Any

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from odio_urdf import *

from moveit import MoveitReconfigurator
from ros2_control import ControllerManager
from utils import *

class RobotConfig:
    def __init__(self, robot_name: str, robot_type: str, parent: str) -> None:
        self.parts = []
        self.name = robot_name
        self.robot_type = robot_type
        self.parent = parent
        self._group = Group()
        self._robot = Robot(name=robot_name)
        self.links = []
        self.joints = []
        self.link_connections = {}
        # WARN: get path via python defaults related to this module?
        self.robot_package_abs_path = get_package_share_directory(self.robot_type)
        self.robot_config = load_yaml_abs(f"{self.robot_package_abs_path}/config/robot_config.yaml")
        self.group_link_joint = {key: [] for key in self.robot_config.keys()}
        self.ros2_control_config = {}

    def add_part(self, part: Any) -> None:
        self.parts.append(part)

    def add_link(self, *args) -> None:
        if len(args) == 1 and isinstance(args[0], Link):
            self._group.extend([args[0]])
        elif len(args) == 3 and isinstance(args[0], int) and isinstance(args[1], dict) and isinstance(args[2], str):
            N, link_config, link_type = args
            link_name = f"{self.name}_link_{N}"
            self.links.append(link_name)
            self.group_link_joint[link_type].append(link_name)
            ret = Link(
                Inertial(
                    Mass(link_config.get("mass", 0)),
                    Inertia(link_config.get("I", [])),
                    Origin(link_config.get("cm", []))
                ),
                Visual(
                    Geometry(
                        Mesh(filename=f"{self.robot_package_abs_path}/meshes/visual/{link_config['mesh_name']}.dae")
                    ),
                ),
                Collision(
                    Geometry(
                        Mesh(filename=f"{self.robot_package_abs_path}/meshes/collision/{link_config['mesh_name']}.stl")
                    )
                ),
                name=link_name
            )
            self._group.extend([ret])

    def add_joint(self, *args) -> None:
        if len(args) == 1 and isinstance(args[0], Joint):
            self._group.extend([args[0]])
        elif len(args) == 3 and isinstance(args[0], int) and isinstance(args[1], dict) and isinstance(args[2], str):
            N, joint_config, link_type = args
            parent = f"{self.name}_link_{N - 1}"
            child = f"{self.name}_link_{N}"
            if parent not in self.link_connections:
                self.link_connections[parent] = []
            self.link_connections[parent].append(child)
            joint_name = f"{self.name}_joint_{N}"
            self.joints.append(joint_name)
            self.group_link_joint[link_type].append(joint_name)
            ret = Joint(
                Parent(link=parent),
                Child(link=child),
                Origin(joint_config.get("origin", [])),
                Axis(xyz=joint_config.get("axis", [])),
                Limit(
                    lower=joint_config.get("limit", [0, 0])[0],
                    upper=joint_config.get("limit", [0, 0])[1],
                    effort=joint_config.get("max_effort", 0),
                    velocity=joint_config.get("max_velocity", 0)
                ),
                type=joint_config.get("type", ""),
                name=joint_name)
            self._group.extend([ret])

    def add_interface(self, interface: str):
        hardware = {
            "gazebo": "ign_ros2_control/IgnitionSystem"
        }
        interface_xml = hardware.get(interface)
        if interface_xml is None:
            raise ValueError(f"Unknown hardware interface [{interface}] where produce ros2_control()")
        
        for joint in self.joints:
            ros2_control = Ros2Control(
                HardwareInterface(xmltext=interface_xml),
                JointInterface(
                    CommandInterface(
                        name="position"),
                    CommandInterface(
                        name="velocity"),
                    StateInterface(name="position"),
                    StateInterface(name="velocity"),
                    StateInterface(name="effort"),
                    name=joint
                ),
                name=joint,
                type="actuator"
            )
            self._group.extend([ros2_control])

    def add_srdf(self):
        MoveitReconfigurator(self, False)

    def add_gripper(self):
        pass

    def add_controller_manager(self, robot_name):
        cm = ControllerManager(self)
        cm.save_to_yaml(f"{robot_name}.yaml",robot_name)

    def add_group(self):
        self._robot.extend([self._group])

    def list_parts(self) -> None:
        print(f"Robot config parts: {', '.join(self.parts)}\n", end="")

    def list_links(self) -> None:
        print(f"Robot links: {', '.join(self.links)}\n", end="")

    def urdf(self) -> Robot:
        return Robot(
            Link(name=self.parent),
            self._group,
            name=self.name)

    def srdf(self) -> Robot:
        return MoveitReconfigurator(self, False).get_srdf()


