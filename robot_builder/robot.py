import importlib
from typing import List, Optional

from odio_urdf import *

from robot_builder.components import Component, JointNode, LinkNode
from robot_builder.moveit import MoveitReconfigurator
from robot_builder.ros2_control import ControllerManager
from robot_builder.utils import *

HARDWARE = {"gazebo": "ign_ros2_control/IgnitionSystem"}

class RobotConfig(Component):
    def __init__(self, robot_name: str, robot_type: str, parent: str = "world") -> None:
        """
        Args:
            robot_name: Name of a robot in scene and also the prefix for ROS workspace
            robot_type: Robot type to find the robot package
            parent: Parent link. Default is world
            absolute_path: Optional. If used then in meshes will be this path
        """
        super().__init__(robot_name)

        # Parts to store parts of builded robot config data
        self.parts: List[str] = []
        self.robot_type: str = robot_type
        self.parent: str = parent
        self.children: List[RobotConfig] = []
        self.components: List[Component] = []

        # Store useuful data to build the robot config
        # TODO: Use dataclasses to store information
        self.links: List[LinkNode] = []
        self.joints: List[JointNode] = []
        self.fixed_links: List[LinkNode] = []
        self.fixed_joints: List[JointNode] = []
        self.robot_joints: List[JointNode] = []
        self.this_robot_joints: List[JointNode] = []

        self.group: Group = Group()
        self.robot_package_abs_path: str = ""
        
        get_package = importlib.import_module("ament_index_python.packages")
        self.robot_package_abs_path = get_package.get_package_share_directory(robot_type)

        # Load robot config which should be in specific place in the package directory
        self.robot_config: dict = load_yaml_abs(f"{self.robot_package_abs_path}/config/robot_config.yaml")
        self.group_link_joint: dict[str, list[str]] = {key: [] for key in self.robot_config.keys()}
        self.ros2_control_config: dict[str, (str | dict | int | float)] = {}

    def add_part(self, part: str) -> None:
        self.parts.append(part)

    def add(self, component: Component) -> None:
        match component:
            case JointNode() if component.parent != "world":
                if component.joint_type != "fixed":
                    self.joints.append(component)
                    self.this_robot_joints.append(component)
                else:
                    self.fixed_joints.append(component)
                self.robot_joints.append(component)
                self.components.append(component)
            case LinkNode():
                self.links.append(component)
                self.components.append(component)
            case RobotConfig():
                self.children.append(component)
            case _:
                pass

    def add_interface(self, interface: str, type: Optional[str] = "actuator") -> None:
        interface_xml = HARDWARE.get(interface)
        if interface_xml is None:
            raise ValueError(f"Unknown hardware interface [{interface}]")
        
        if type == "actuator":
            for i, joint in enumerate(self.joints):
                if joint.joint_type != "fixed":
                    ros2_control = Ros2Control(
                        HardwareInterface(Plugin(xmltext=interface_xml)),
                        JointInterface(
                            CommandInterface(name="position"),
                            CommandInterface(name="velocity"),
                            *[StateInterface(name=name) for name in ["position", "velocity", "effort"]],
                            name=joint.name
                        ),
                        name=str(i) + "_interface",
                        type=type
                    )
                    self.group.extend([ros2_control])
        elif type == "system": #WARN: Dangerous it's massive hardcode here
            ros2_control = Ros2Control(
                Hardwareinterface(Plugin(xmltext=interface_xml)),

                name=self.component_name,
                type=type
            )

        else:
            raise RuntimeError("Interface type is not supported yet! Please use from list [actuator, system]")



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
        cm.save_to_yaml(f"{robot_name}.yaml", robot_name)

    # def add_group(self) -> None:
    #     self.robot.extend([self.group])

    def add_simulation(self, simulator: str | None) -> None:
        if simulator in HARDWARE:
            simulation = Gazebo(
                Plugin(
                    Parameters(
                        xmltext=f"{self.robot_package_abs_path}/config/{self.component_name}.yaml"
                    ),
                    # Ros(Namespace(xmltext=f"/{self.component_name}")), #TODO: enable Namespace
                    filename="libign_ros2_control-system.so",
                    name="ign_ros2_control::IgnitionROS2ControlPlugin"))
            self.group.extend([simulation])
        else:
            raise RuntimeError("Please set a simulator or call the ros2_control method first")

    def merge_children(self) -> None:
        for child in self.children:
            self.links.extend(child.links)
            self.robot_joints.extend(child.robot_joints)
            self.joints.extend(child.joints)
            self.components.extend(child.components)

    def list_parts(self) -> None:
        print(f"Robot config parts: {', '.join(self.parts)}\n", end="")

    def list_links(self) -> None:
        print("Robot links: ")
        for link in self.links:
            print(link.name)

    def list_joints(self) -> None:
        print("Robot joints: ")
        for joint in self.joints:
            print(joint.name)

    def get_geometry(self, visual: bool = False, collision: bool = False) -> tuple[List, List]:
        """
        Args:
            visual: if true then store visual mesh paths 
            collision:  if true then store collision mesh paths

        Returns:
            visual_mesh_path_list
            collision_mesh_path_list
        """
        visual_mesh_path_list = []
        collision_mesh_path_list = []
        for link in self.links:
            if visual and link.link_config:
                visual_mesh_path_list.append(link.visual_mesh_package_path)
            if collision and link.link_config:
                collision_mesh_path_list.append(link.collision_mesh_package_path)
        return (visual_mesh_path_list, collision_mesh_path_list)

    def urdf(self) -> Robot:
        for component in self.components:
            self.group.extend([component.get_urdf()])
        return Robot(self.group, name=self.component_name)

    def srdf(self) -> Robot:
        return MoveitReconfigurator(self, False).get_srdf()
