from dataclasses import dataclass, field
from typing import TYPE_CHECKING

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
    def joint_names(self):
        return [j.name for j in self.joints]

    @property
    def actuated_joints(self):
        return self._actuated_joints

    @property
    def actuated_dof_indices(self):
        return self._actuated_dof_indices

    @property
    def actuated_joint_indices(self):
        return self._actuated_joint_indices

    @property
    def actuated_joint_names(self):
        return [j.name for j in self._actuated_joints]

    @property
    def num_actuated_joints(self):
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
    def base_link(self):
        return self._base_link

    @property
    def ee_link(self):
        return self._ee_link

    @property
    def get_gripper_joint_names(self) -> list:
        return self._gripper_joint_names

    @property
    def get_gripper_joints(self) -> list[Joint]:
        return self._gripper_joints

    @property
    def get_gripper_mimic_joint_name(self):
        return self._gripper_mimic_joint_name

    def _determine_base_link(self) -> str | None:
        link_names = [link.name for link in self.links]
        for j in self.joints:
            if j.child is not None:
                link_names.remove(j.child)
        if len(link_names) == 0:
            return None
        return link_names[0]

    def _determine_ee_link(self) -> str | None:
        link_names = [link.name for link in self.links]
        return link_names[-1]

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
        self._gripper_link = []
        self._gripper_link_names = []
        self._gripper_joints = []
        self._gripper_joint_names = []
        for link in self.links:
            name = link.name.split("_")
            if gripper_keyname in name:
                self._gripper_link.append(link)
                self._gripper_link_names.append(link.name)
        for joint in self.joints:
            name = joint.name.split("_")
            if gripper_keyname in name:
                self._gripper_joints.append(joint)
                self._gripper_joint_names.append(joint.name)
                if joint.mimic:
                    mimic_actuated_joint = joint.mimic.joint
                    if mimic_actuated_joint is not None:
                        self.joints.remove(self._joint_map[mimic_actuated_joint])
                        self._gripper_mimic_joint_name = joint.mimic.joint
                        self._gripper_mimic_joint = self._joint_map[
                            mimic_actuated_joint
                        ]
                    else:
                        raise ValueError("Determine mimic joint name")

    def extend(self, other: "Robot"):
        self.links.extend(other.links)
        self.joints.extend(other.joints)
        self.materials.extend(other.materials)
        self.gazebo.extend(other.gazebo)
        self.control.extend(other.control)

    def init(self, gripper_prefix: str):
        self._create_maps()
        self._split_gripper(gripper_prefix)
        self._update_actuated_joints()
        self._base_link = self._determine_base_link()
        self._ee_link = self._determine_ee_link()
        self._cfg = self.zero_cfg

        self.parallel_gripper = False
        self.multifinger_gripper = False

        if self._gripper_joint_names and self._gripper_mimic_joint_name:
            self.parallel_gripper = True

        elif self._gripper_joint_names and not self._gripper_mimic_joint_name:
            self.multifinger_gripper = True

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_robot(config, self)
