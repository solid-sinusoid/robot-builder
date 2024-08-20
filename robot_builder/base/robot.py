from dataclasses import field
from pydantic.dataclasses import dataclass

# from loguru import logger
import numpy as np

from .base import Component, Visitor
from .ros2_control_interface import Ros2Control
from .joint import Joint
from .link import Link, Material


@dataclass(eq=False)
class Robot(Component):
    name: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    materials: list[Material] = field(default_factory=list)
    gazebo: list[str] = field(default_factory=list)
    control: list[Ros2Control] = field(default_factory=list)

    @property
    def link_map(self) -> dict:
        return self._link_map

    @property
    def joint_map(self) -> dict:
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
    def zero_cfg(self):
        return np.zeros(self.num_dofs)

    @property
    def cfg(self):
        return self._cfg

    @property
    def base_link(self):
        return self._base_link

    def _determine_base_link(self) -> str | None:
        link_names = [l.name for l in self.links]
        for j in self.joints:
            if j.child is not None:
                link_names.remove(j.child)
        if len(link_names) == 0:
            return None
        return link_names[0]

    def _create_maps(self):
        self._material_map = {m.name: m for m in self.materials}
        self._joint_map = {j.name: j for j in self.joints}
        self._link_map = {l.name: l for l in self.links}

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

    # def __eq__(self, other):
    #     if not isinstance(other, Robot):
    #         return NotImplemented
    #     return (
    #         self.name == other.name
    #         and set(self.links) == set(other.links)
    #         and set(self.joints) == set(other.joints)
    #         and set(self.materials) == set(other.materials)
    #         and set(self.gazebo) == set(other.gazebo)
    #         and set(self.control) == set(other.control)
    #     )

    def extend(self, other: 'Robot'):
        self.links.extend(other.links)
        self.joints.extend(other.joints)
        self.materials.extend(other.materials)
        self.gazebo.extend(other.gazebo)
        self.control.extend(other.control)

    def visit(self, config, visitor: Visitor):
        visitor.visit_robot(config, self)
        self._create_maps()
        self._update_actuated_joints()
        self._base_link = self._determine_base_link()
        self._cfg = self.zero_cfg
