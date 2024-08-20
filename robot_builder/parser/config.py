import os
from typing import Type
import six
import numpy as np
from functools import partial
from robot_builder.base import *
from loguru import logger
import toml

from lxml import etree

# _logger = logging.getLogger(__name__)

class RobotConfigParser(Visitor):
    def __init__(
        self,
        robot: Robot,
        load_meshes: bool = True,
        load_collision_meshes: bool = True,
        filename_handler=None,
        mesh_dir: str = "",
    ):
        if filename_handler is None:
            self._filename_handler = partial(filename_handler_magic, dir=mesh_dir)
        else:
            self._filename_handler = filename_handler

        self.robot = robot

    @staticmethod
    def load(fname_or_file, **kwargs):
        if isinstance(fname_or_file, six.string_types):
            if not os.path.isfile(fname_or_file):
                raise ValueError("{} is not a file".format(fname_or_file))

            if not "mesh_dir" in kwargs:
                kwargs["mesh_dir"] = os.path.dirname(fname_or_file)

        try:
            config = toml.load(fname_or_file)
            if "robot_name" in kwargs:
                robot_names = kwargs["robot_name"]
            else:
                robots = config.get("robots", "")
                robot_names = robots.keys()
            # parser = etree.XMLParser(remove_blank_text=True)
            # tree = etree.parse(fname_or_file, parser=parser)
            # xml_root = tree.getroot()
        except Exception as e:
            logger.error(e)
            raise ValueError("Something wrong")

        robots_lst = []
        for r in robot_names:
            robot = Robot(name=r)  # Adjust as needed
            robot_config = RobotConfigParser(robot)

            # robot_config.visit_robot(config, robot)

            # Process the robot using Visitor pattern
            robot.visit(config, robot_config)
            robots_lst.append(robot)

        return robots_lst[0]


    def visit_mimic(self, config: etree._Element | dict, element: Mimic):
        if not isinstance(config, dict):
            return NotImplemented

        element.joint = config.get("joint")
        element.multiplier = config.get("multiplier", 1.0)
        element.offset = config.get("offset", 0.0)

    def visit_safety_controller(self, config: etree._Element | dict, element: SafetyController):
        if not isinstance(config, dict):
            return NotImplemented

        element.soft_lower_limit = config.get("soft_lower_limit")
        element.soft_upper_limit = config.get("soft_upper_limit")
        element.k_position = config.get("k_position")
        element.k_velocity = config.get("k_velocity")

    def visit_box(self, config: etree._Element | dict, element: Box):
        if not isinstance(config, dict):
            return NotImplemented
        box_size = config.get("box")
        match box_size:
            case list():
                element.size = np.array(box_size)
            case dict():
                element.size = np.array([
                    box_size.get('x'),
                    box_size.get('y'),
                    box_size.get('z')
                ])
                

    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder):
        if not isinstance(config, dict):
            return NotImplemented
        element.radius = config.get("radius")
        element.length = config.get("length")

    def visit_sphere(self, config: etree._Element | dict, element: Sphere):
        if not isinstance(config, dict):
            return NotImplemented
        element.radius = float(config["radius"])

    def visit_scale(self, config: etree._Element | dict, element: Scale):
        if not isinstance(config, dict):
            return NotImplemented
        element.scale = None
        value = config.get("value")
        match value:
            case list():
                element.scale = np.array(value)
            case dict():
                element.scale = np.array([
                    value.get('x'), value.get('y'), value.get('z')
                ])
            case float():
                element.scale = value
            case _:
                element.scale = 1.0

    def visit_mesh(self, config: etree._Element | dict, element: Mesh):
        if not isinstance(config, dict):
            return NotImplemented

        element.filename = config.get("filename")
        self.visit_and_add(config.get("scale", None), Scale, element)

    def visit_geometry(self, config: etree._Element | dict, element: Geometry):
        if not isinstance(config, dict):
            return NotImplemented

        match config:
            case {"mesh": mesh_cfg}:
                mesh = Mesh()
                mesh.visit(mesh_cfg, self)
                element.add(mesh)
            case {"box": box_cfg}:
                box = Box()
                box.visit(box_cfg, self)
            case {"cylinder": cylinder_cfg}:
                cylinder = Cylinder()
                cylinder.visit(cylinder_cfg, self)
            case _:
                raise ValueError(f"Unknown geometry type: {config}")

    def visit_origin(self, config: etree._Element | dict, element: Origin):
        if not isinstance(config, dict):
            return NotImplemented

        position = config.get("position", {})
        orientation = config.get("orientation", {})

        element.xyz = np.array([
            position.get("x", 0.0),
            position.get("y", 0.0),
            position.get("z", 0.0)
        ])
        element.rpy = np.array([
            orientation.get("x", 0.0),
            orientation.get("y", 0.0),
            orientation.get("z", 0.0)
        ])

    def visit_color(self, config: etree._Element | dict, element: Color):
        if not isinstance(config, dict):
            return NotImplemented
        rgba = config.get("rgba", [1, 1, 1, 1])
        element.rgba = np.array(rgba)

    def visit_texture(self, config: etree._Element | dict, element: Texture):
        if not isinstance(config, dict):
            return NotImplemented
        element.filename = config.get("filename", None)

    def visit_material(self, config: etree._Element | dict, element: Material):
        if not isinstance(config, dict):
            return NotImplemented
        element.name = config.get("name")

        self.visit_and_add(config.get("color", None), Color, element)
        self.visit_and_add(config.get("texture", None), Texture, element)

    def visit_visual(self, config: etree._Element | dict, element: Visual):
        if not isinstance(config, dict):
            return NotImplemented
        element.name = config.get("name", None)

        self.visit_and_add(config.get("origin", None), Origin, element)
        self.visit_and_add(config.get("visual", None), Geometry, element)
        self.visit_and_add(config.get("material", None), Material, element)

    def visit_collision(self, config: etree._Element | dict, element: Collision):
        if not isinstance(config, dict):
            return NotImplemented
        element.name = config.get("name", None)

        self.visit_and_add(config.get("origin", None), Origin, element)
        self.visit_and_add(config.get("collision", None), Geometry, element)

    def visit_inertia(self, config: etree._Element | dict, element: Inertia):
        if not isinstance(config, dict):
            return NotImplemented
        element.inertia_tensor =  np.array(
            [
                [
                    config.get("ixx", 1.0),
                    config.get("ixy", 0.0),
                    config.get("ixz", 0.0),
                ],
                [
                    config.get("ixy", 0.0),
                    config.get("iyy", 1.0),
                    config.get("iyz", 0.0),
                ],
                [
                    config.get("ixz", 0.0),
                    config.get("iyz", 0.0),
                    config.get("izz", 1.0),
                ],
            ],
            dtype=np.float64,
        )

    def visit_mass(self, config: etree._Element | dict, element: Mass):
        if not isinstance(config, dict):
            return NotImplemented
        element.value = config.get("value", None)

    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        if not isinstance(config, dict):
            return NotImplemented

        self.visit_and_add(config.get("origin", None), Origin, element)
        self.visit_and_add(config.get("inertia", None), Inertia, element)
        self.visit_and_add(config.get("mass", None), Mass, element)

    def visit_link(self, config: etree._Element | dict, element: Link):
        if not isinstance(config, dict):
            return NotImplemented
        
        self.visit_and_add(config.get("inertial", None), Inertial, element)
        self.visit_and_add(config.get("geometry", None), Visual, element)
        self.visit_and_add(config.get("geometry", None), Collision, element)

    def visit_axis(self, config: etree._Element | dict, element: Axis):
        if not isinstance(config, dict):
            return NotImplemented
        xyz = config.get("xyz", [1, 0, 0])
        element.axis = np.array(xyz)


    def visit_limit(self, config: etree._Element | dict, element: Limit):
        if not isinstance(config, dict):
            return NotImplemented
        element.effort=config.get("max_effort", None)
        element.velocity=config.get("max_velocity", None)
        element.lower=config.get("lower", None)
        element.upper=config.get("upper", None)

    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics):
        if not isinstance(config, dict):
            return NotImplemented
        element.damping = config.get("damping", None)
        element.friction = config.get("friction", None)

    def visit_calibration(self, config: etree._Element | dict, element: Calibration):
        if not isinstance(config, dict):
            return NotImplemented
        return super().visit_calibration(config, element)

    def visit_joint(self, config: etree._Element | dict, element: Joint):
        if not isinstance(config, dict):
            return NotImplemented
        element.type = config.get("type", "fixed")
        # For fixed link origin is only needed
        if not config.get("axis") and element.type == "fixed":
            self.visit_and_add(config, Origin, element)
        else:
            self.visit_and_add(config.get("origin", None), Origin, element)
            self.visit_and_add(config.get("axis", None), Axis, element)
            self.visit_and_add(config.get("limit", None), Limit, element)
            self.visit_and_add(config.get("dynamics", None), Dynamics, element)
            self.visit_and_add(config.get("mimic", None), Mimic, element)
            self.visit_and_add(config.get("calibration", None), Calibration, element)
            self.visit_and_add(config.get("safety_controller", None), SafetyController, element)

        element.parent = self.robot.links[-2].name
        element.child = self.robot.links[-1].name


    def visit_robot(self, config: etree._Element | dict, element: Robot):
        if not isinstance(config, dict):
            return NotImplemented

        links = config.get("links", {})
        if not links:
            raise ValueError("Please provide links information")

        try:
            parent = config["robots"][element.name]["parent"]
        except Exception as e:
            logger.error(f"Parent link of Robot isn't set using default value: world")
            parent = "world"

        link = Link(name=f"{parent}")
        element.add(link)
        for i, l_name in enumerate(config["robots"][element.name]["links"]):
            link = Link(name=f"{element.name}_{l_name}_{i}")
            element.add(link)
            if l_name in config["links"]:
                link.visit(config["links"][l_name], self)
            else:
                link.visit(config["anchor"][l_name], self)

            #TODO: A better way to do this choise
            # If i = 0 then create joint from robot parent to robot root
            if i == 0:
                joint = Joint(name=f"{element.name}_{parent}_{link.name}_{i}_joint")
                element.add(joint)
                joint.visit(config["anchor"][parent], self)
            else:
                if l_name in config["links"]:
                    joint = Joint(name=f"{element.name}_{parent}_{link.name}_{i}_joint")
                    element.add(joint)
                    joint.visit(config["links"][l_name]["joint"], self)
                elif l_name in config["anchor"]:
                    joint = Joint(name=f"{element.name}_{parent}_{link.name}_{i}_joint")
                    element.add(joint)
                    joint.visit(config["anchor"][l_name], self)

            print(i)

            parent = link.name
        # for j in config.findall("joint"):
        #     joint = Joint(name=j.attrib["name"])
        #     joint.visit(j, self)
        #     element.add(joint)

        # for m in config.findall("material"):
        #     robot.materials.append(URDF._visit_material(m))

    def visit_and_add(self, config: etree._Element | dict, child: Type[Component], parent: Component):
        if not isinstance(config, dict):
            return NotImplemented
        component = child()
        parent.add(component)
        component.visit(config, self)

