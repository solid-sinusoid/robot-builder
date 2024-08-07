from abc import ABC, abstractmethod
from odio_urdf import Element, Mimic
from odio_urdf import Link, Inertia, Inertial, Mass, Origin, Mesh, Visual, Collision, Geometry
from odio_urdf import Joint, Parent, Child, Axis, Limit
from typing import Optional
import shutil

class Component:
    def __init__(self, name: str) -> None:
        self.component_name = name

    @abstractmethod
    def get_urdf(self) -> Element:
        pass

class LinkNode(Component):
    def __init__(self, 
                 N: int | None, 
                 robot_name: str | None, 
                 link_config: dict | None, 
                 link_name: str | None = None, 
                 package_path: str | None = None,
                 save_geometry_path: str | None = None) -> None:
        if link_name == "world":
            name = link_name
        elif link_name in ("tool0", "grasp_point"):
            name = f"{robot_name}_{link_name}"
        else:
            name = f"{robot_name}_{link_name}_{N}"
        super().__init__(name)
        self.link_name = link_name
        self.link_config = link_config
        self.package_path = package_path
        if link_config is not None and link_config.get("mesh_name", ""):
            self.visual_mesh_package_path = f"{self.package_path}/meshes/visual/{self.link_config['mesh_name']}.dae"
            self.collision_mesh_package_path = f"{self.package_path}/meshes/collision/{self.link_config['mesh_name']}.stl"
            if save_geometry_path is not None:
                self.visual_mesh_save_path = f"{save_geometry_path}/{self.link_config['mesh_name']}.dae"
                self.collision_mesh_save_path = f"{save_geometry_path}/{self.link_config['mesh_name']}.stl"
            else:
                self.visual_mesh_save_path = self.visual_mesh_package_path
                self.collision_mesh_save_path = self.collision_mesh_package_path

        self.robot_name = robot_name

    @property
    def name(self) -> str:
        return self.component_name

    def get_urdf(self) -> Element:
        if not self.link_config:
            return Link(name=self.component_name)

        ret = Link(
            Inertial(
                Mass(self.link_config.get("mass", 0)),
                Inertia(self.link_config.get("I", [])),
                Origin(self.link_config.get("cm", []))
            ),
            Visual(
                Geometry(
                    Mesh(filename=f"file://{self.visual_mesh_save_path}")
                ),
            ),
            Collision(
                Geometry(
                    Mesh(filename=f"file://{self.visual_mesh_save_path}")
                )
            ),
            name=self.component_name
        )
        return ret

class JointNode(Component):
    _mimic_joint_name = []
    def __init__(self, N: Optional[int], 
                 parent: str, child: str, 
                 joint_config: Optional[dict] = None) -> None:
        name = f"{parent}_{child}"
        self.parent = parent
        self.joint_type = joint_config.get("type", "")
        self.child = child
        self.joint_config = joint_config
        self.multiplier = joint_config.get("multiplier", 0)
        self.joint_limit: List[float] = joint_config.get("joint_limit", [0, 0])
        self.max_velocity = joint_config.get("max_velocity", 0)
        self.mimic = joint_config.get("mimic", False)
        self.has_mimic = joint_config.get("has_mimic", False)
        if self.has_mimic:
            JointNode._mimic_joint_name.append(name)
        super().__init__(name)

    @property
    def name(self) -> str:
        return self.component_name

    def get_urdf(self) -> Element:
        if self.joint_type == "fixed":
            ret = Joint(
                Parent(link=self.parent),
                Child(link=self.child),
                Origin(self.joint_config.get("origin", [0,0,0,0,0,0])),
                name=self.component_name,
                type=self.joint_type)

        elif self.joint_config:
            ret = Joint(
                Parent(link=self.parent),
                Child(link=self.child),
                Origin(self.joint_config.get("origin", [])),
                Axis(xyz=self.joint_config.get("axis", [])),
                Limit(
                    lower=self.joint_config.get("limit", [0, 0])[0],
                    upper=self.joint_config.get("limit", [0, 0])[1],
                    effort=self.joint_config.get("max_effort", 0),
                    velocity=self.joint_config.get("max_velocity", 0)
                ),
                type=self.joint_type,
                name=self.component_name)
            if self.mimic:
                mimic = Mimic(joint=JointNode._mimic_joint_name[0], 
                              multiplier=self.multiplier)
                ret.extend([mimic])
        else:
            ret = Joint()
            assert ValueError("Please descripe joint_config")
        return ret
