from typing import List
from odio_urdf import *
from component import Component


class LinkNode(Component):
    def __init__(self, name: str, link_config: dict, link_type: str, package_abs_path: str) -> None:
        super().__init__(name)
        self.link_config = link_config
        self.link_type = link_type
        self.package_abs_path = package_abs_path

    @property
    def name(self) -> str:
        return self.component_name

    def get_urdf(self) -> Element:
        ret = Link(
            Inertial(
                Mass(self.link_config.get("mass", 0)),
                Inertia(self.link_config.get("I", [])),
                Origin(self.link_config.get("cm", []))
            ),
            Visual(
                Geometry(
                    Mesh(filename=f"{self.package_abs_path}/meshes/visual/{self.link_config['mesh_name']}.dae")
                ),
            ),
            Collision(
                Geometry(
                    Mesh(filename=f"{self.package_abs_path}/meshes/collision/{self.link_config['mesh_name']}.stl")
                )
            ),
            name=self.component_name
        )
        return ret

class JointNode(Component):
    def __init__(self, name: str, parent: str, child: str, joint_config: dict) -> None:
        super().__init__(name)
        self.parent = parent
        self.child = child
        self.joint_config = joint_config

    def get_urdf(self) -> Element:
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
            type=self.joint_config.get("type", ""),
            name=self.component_name)
        return ret

class Composite(Component):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.children: List[Component] = []
        self.group: Group = Group()

    def add(self, component: Component) -> None:
        self.children.append(component)

    def remove(self, component: Component) -> None:
        self.children.remove(component)

    def get_urdf(self) -> Element:
        for child in self.children:
            self.group.extend([child.get_urdf()])
        return self.group

# Пример использования
if __name__ == "__main__":
    robot = Composite("Robot")

    # Создаем Link1 и Link2
    link1_config = {
        "mass": 1.0,
        "I": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "cm": [0.0, 0.0, 0.0],
        "mesh_name": "link1_mesh"
    }
    link1 = LinkNode("Link1", link1_config, "type1", "/path/to/package")

    link2_config = {
        "mass": 1.0,
        "I": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "cm": [0.0, 0.0, 0.0],
        "mesh_name": "link2_mesh"
    }
    link2 = LinkNode("Link2", link2_config, "type2", "/path/to/package")

    # Добавляем Link1
    robot.add(link1)

    # Добавляем Joint1
    joint1_config = {
        "origin": [0.0, 0.0, 0.0],
        "axis": [0.0, 0.0, 1.0],
        "limit": [0, 0],
        "max_effort": 100,
        "max_velocity": 1.0,
        "type": "revolute"
    }
    joint1 = JointNode("Joint1", link1.name, link2.name, joint1_config)
    robot.add(joint1)

    # Добавляем Link2
    robot.add(link2)

    print(robot.get_urdf())
