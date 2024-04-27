from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from odio_urdf import *

from robot_builder.components import Component, JointNode, LinkNode
from robot_builder.moveit import MoveitReconfigurator
from robot_builder.ros2_control import ControllerManager
from robot_builder.utils import *


HARDWARE = {"gazebo": "ign_ros2_control/IgnitionSystem"}

class RobotConfig(Component):
    def __init__(self, robot_name: str, robot_type: str, parent: str) -> None:
        super().__init__(robot_name)
        self.parts: List[str] = []
        self.robot_type: str = robot_type
        self.parent: str = parent
        self.children: List[RobotConfig] = []
        self.components: List[Component] = []

        self.links: List[LinkNode] = []
        self.joints: List[JointNode] = []
        self.fixed_links: List[LinkNode] = []
        self.fixed_joints: List[JointNode] = []
        self.robot_joints: List[JointNode] = []
        self.this_robot_joints: List[JointNode] = []

        self.group: Group = Group()
        self.robot_package_abs_path: str = get_package_share_directory(self.robot_type)
        self.robot_config: dict = load_yaml_abs(f"{self.robot_package_abs_path}/config/robot_config.yaml")
        self.group_link_joint: Dict[str, List[str]] = {key: [] for key in self.robot_config.keys()}
        self.ros2_control_config: Dict[str, Union[str, dict]] = {}

    def add_part(self, part: str) -> None:
        self.parts.append(part)

    def add(self, component: Component) -> None:
        if isinstance(component, JointNode) and component.parent != "world":
            if component.joint_type != "fixed":
                self.joints.append(component)
                self.this_robot_joints.append(component)
            else:
                self.fixed_joints.append(component)
            self.robot_joints.append(component)
        elif isinstance(component, LinkNode):
            self.links.append(component)
        elif isinstance(component, RobotConfig):
            self.children.append(component)
        
        if isinstance(component, (LinkNode, JointNode)):
            self.components.append(component)

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

    def add_group(self) -> None:
        self.robot.extend([self.group])

    def add_simulation(self, simulator: str) -> None:
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

    def urdf(self) -> Robot:
        for component in self.components:
            self.group.extend([component.get_urdf()])
        return Robot(self.group, name=self.component_name)

    def srdf(self) -> Robot:
        return MoveitReconfigurator(self, False).get_srdf()
