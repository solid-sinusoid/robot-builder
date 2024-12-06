from dataclasses import dataclass, field

from loguru import logger
from lxml import etree

from ..base.base import Visitor
from ..base.component import Component
from .gazebo import Gazebo
from .joint import Joint
from .link import Link, Material
from .ros2_control_interface import Ros2Control


@dataclass(eq=False)
class Robot(Component):
    name: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    materials: list[Material] = field(default_factory=list)
    control: list[Ros2Control] = field(default_factory=list)
    gazebo: list[Gazebo] = field(default_factory=list)

    @property
    def link_map(self) -> dict[str, Link]:
        return self._link_map

    @property
    def link_names(self) -> list[str]:
        return [link.name for link in self.links]

    @property
    def control_map(self) -> dict[str, Ros2Control]:
        return self._control_map

    @property
    def control_names(self) -> list[str]:
        return [control.name for control in self.control]

    @property
    def joint_map(self) -> dict[str, Joint]:
        return self._joint_map

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.joints]

    @property
    def actuated_joints(self) -> list[Joint]:
        return self._actuated_joints

    @property
    def actuated_dof_indices(self) -> list[int]:
        return self._actuated_dof_indices

    @property
    def actuated_joint_indices(self) -> list[int]:
        return self._actuated_joint_indices

    @property
    def actuated_joint_names(self) -> list[str]:
        return [j.name for j in self._actuated_joints]

    @property
    def num_actuated_joints(self) -> int:
        return len(self.actuated_joints)

    @property
    def num_dofs(self):
        total_num_dofs = 0
        for j in self._actuated_joints:
            if j.type in ["revolute", "prismatic", "continuous"]:
                total_num_dofs += 1
            elif j.type == "floating":
                total_num_dofs += 6
            elif j.type == "planar":
                total_num_dofs += 2
        return total_num_dofs

    @property
    def zero_cfg(self) -> list[float]:
        return [0] * self.num_dofs

    @property
    def cfg(self) -> list[float]:
        return self._cfg

    @property
    def base_link(self) -> str:
        return self._base_link

    @property
    def ee_link(self) -> str:
        return self._ee_link

    @property
    def get_gripper_joint_names(self) -> list[str]:
        return self._gripper_joint_names

    @property
    def get_gripper_joints(self) -> list[Joint]:
        return self._gripper_joints

    # @property
    # def get_gripper_mimic_joint_name(self) -> str | None:
    #     return self._gripper_mimic_joint_name

    @property
    def gripper_actuated_joints(self) -> list[Joint]:
        return self._gripper_actuated_joints

    @property
    def gripper_actuated_joint_names(self) -> list[str]:
        return [j.name for j in self._gripper_actuated_joints]

    def _determine_base_link(self) -> str:
        link_names = {link.name for link in self.links if link.name != "world"}
        child_links = {
            j.child for j in self.joints if j.child is not None and j.parent != "world"
        }

        base_links = link_names - child_links
        if not base_links:
            raise RuntimeError("Please define link without parents in your URDF")

        return next(iter(base_links))

    def _determine_ee_link(self, ee_link_name: str = "") -> str:
        # If an end-effector link name is specified, return it from the link_map
        if ee_link_name and ee_link_name in self.link_map:
            return self.link_map[ee_link_name].name

        # Get a list of all link names and gripper link names
        link_names = [link.name for link in self.links]
        gripper_links = [link.name for link in self._gripper_links]

        # Check the last gripper link, if it exists
        if gripper_links:
            last_gripper_link = gripper_links[-1]
            last_gripper_joint_name = self.link_map[last_gripper_link].name if last_gripper_link in self.link_map else None

            # Verify that the associated joint exists and is not fixed
            if last_gripper_joint_name and last_gripper_joint_name in self.joint_map:
                joint_type = self.joint_map[last_gripper_joint_name].type
                if joint_type and joint_type != "fixed":
                    return last_gripper_link

        # If no suitable gripper link is found, return the last link of the robot
        return link_names[-1] if link_names else ""
        


    def _create_maps(self):
        self._control_map = {control.name: control for control in self.control}
        self._material_map = {material.name: material for material in self.materials}
        self._joint_map = {joint.name: joint for joint in self.joints}
        self._link_map = {link.name: link for link in self.links}

    def _update_actuated_joints(self):
        self._actuated_joints = []
        self._actuated_joint_indices = []
        self._actuated_dof_indices = []
        dof_indices_cnt = 0
        for i, j in enumerate(self.joints):
            if j.mimic is None and j.type != "fixed":
                self._actuated_joints.append(j)
                self._actuated_joint_indices.append(i)
                if j.type in ["prismatic", "revolute", "continuous"]:
                    self._actuated_dof_indices.append([dof_indices_cnt])
                    dof_indices_cnt += 1
                elif j.type == "floating":
                    self._actuated_dof_indices.append(
                        [dof_indices_cnt, dof_indices_cnt + 1, dof_indices_cnt + 2]
                    )
                    dof_indices_cnt += 3
                elif j.type == "planar":
                    self._actuated_dof_indices.append(
                        [dof_indices_cnt, dof_indices_cnt + 1]
                    )
                    dof_indices_cnt += 2

    def _split_gripper(self, gripper_keyname: str | None = None):
        self._gripper_actuated_joints: list[Joint] = []
        self._gripper_joint_names: list[str] = []
        self._gripper_links: list[Link] = []
        self._gripper_joints: list[Joint] = []
        self._gripper_mimiced_actuated_joints: list[Joint] = []

        for joint in self.joints:
            if not joint.mimic and joint.type == "prismatic":
                self._gripper_actuated_joints.extend([joint])

            # Identify gripper joints based on key name
            name_parts = joint.name.split("_")
            if gripper_keyname in name_parts and not joint.type == "revolute":
                self._gripper_joints.append(joint)


        if len(self._gripper_actuated_joints) == 1:
            self.parallel_gripper = True
            logger.info("Gripper type is parallel so the position_controllers/GripperActionController will be selected")
        elif len(self._gripper_actuated_joints) > 1:
            self.multifinger_gripper = True
            logger.info("Gripper type is multifinger so the joint_trajectory_controller will be selected")
            raise ValueError("Check your configuration multifinger gripper currently not supported")
        else:
            logger.error("Gripper has not actuated joints")

        # Remove gripper joints from the robot's joints
        self.joints = [
            joint
            for joint in self.joints
            if joint not in self._gripper_joints
        ]

        # Remove gripper links from the robot's links
        for link in self.links:
            name_parts = link.name.split("_")
            if gripper_keyname in name_parts:
                self._gripper_links.append(link)

        # Update links after removal
        self.links = [link for link in self.links if link not in self._gripper_links]

    def extend(self, other: "Robot"):
        self.links.extend(other.links)
        self.joints.extend(other.joints)
        self.materials.extend(other.materials)
        self.gazebo.extend(other.gazebo)
        self.control.extend(other.control)

    def init(self, gripper_prefix: str, ee_link_name: str = ""):
        self.parallel_gripper = False
        self.multifinger_gripper = False
        self._create_maps()
        self._split_gripper(gripper_prefix)
        self._update_actuated_joints()
        self._base_link = self._determine_base_link()
        self._ee_link = self._determine_ee_link(ee_link_name)
        self._cfg = self.zero_cfg

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor

        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_robot(config, self)
