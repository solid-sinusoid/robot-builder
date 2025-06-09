import os
from functools import partial

import numpy as np
import six
from loguru import logger
from lxml import etree

from ..base.robot import RobotVisitor
from ..elements.gazebo import Gazebo
from ..elements.joint import (
    Axis,
    Calibration,
    Dynamics,
    Joint,
    Limit,
    Mimic,
    SafetyController,
)
from ..elements.link import (
    Box,
    Collision,
    Color,
    Cylinder,
    Geometry,
    Inertia,
    Inertial,
    Link,
    # Mass,
    Material,
    Mesh,
    Origin,
    Scale,
    Sphere,
    Texture,
    Visual,
)
from ..elements.robot import Robot
from ..elements.ros2_control_interface import Ros2Control
from ..utils import filename_handler_magic, str2float
from .gazebo import GazeboUrdfParser
from .ros2_control import Ros2ControlUrdfParser


class URDF_parser(RobotVisitor):
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
    def load_string(robot_xml_string: str, **kwargs):
        try:
            parser = etree.XMLParser(remove_blank_text=True)
            tree = etree.parse(six.StringIO(robot_xml_string), parser=parser)
            xml_root = tree.getroot()
        except Exception as e:
            logger.error(e)
            logger.error("Using different parsing approach.")
            events = ("start", "end", "start-ns", "end-ns")
            xml = etree.iterparse(
                six.StringIO(robot_xml_string), recover=True, events=events
            )

            # Iterate through all XML elements
            for action, elem in xml:
                # Skip comments and processing instructions,
                # because they do not have names
                if not (
                    isinstance(elem, etree._Comment)
                    or isinstance(elem, etree._ProcessingInstruction)
                ):
                    # Remove a namespace URI in the element's name
                    # elem.tag = etree.QName(elem).localname
                    if action == "end" and ":" in elem.tag:
                        elem.getparent().remove(elem)

            if xml.root is None:
                raise RuntimeError("XML root is empty")
            xml_root = xml.root
        # Remove comments
        etree.strip_tags(xml_root, etree.Comment)
        etree.cleanup_namespaces(xml_root)
        # Create and configure the URDF_visitor
        robot = Robot(name=xml_root.attrib["name"].__str__())  # Adjust as needed
        urdf_parser = URDF_parser(
            robot=robot,
            mesh_dir=kwargs.get("mesh_dir", ""),
            filename_handler=kwargs.get("filename_handler", None),
        )

        # Process the robot using Visitor pattern
        robot.visit(xml_root, urdf_parser)
        robot.init(kwargs.get("base_link_name", ""), kwargs.get("ee_link_name", ""), kwargs.get("merged_gripper_joint", False))
        return robot

    @staticmethod
    def load(fname_or_file, **kwargs):
        if isinstance(fname_or_file, six.string_types):
            if not os.path.isfile(fname_or_file):
                raise ValueError("{} is not a file".format(fname_or_file))

            if "mesh_dir" not in kwargs:
                kwargs["mesh_dir"] = os.path.dirname(fname_or_file)

        try:
            parser = etree.XMLParser(remove_blank_text=True)
            tree = etree.parse(fname_or_file, parser=parser)
            xml_root = tree.getroot()
        except Exception as e:
            logger.error(e)
            logger.error("Using different parsing approach.")

            events = ("start", "end", "start-ns", "end-ns")
            xml = etree.iterparse(fname_or_file, recover=True, events=events)

            # Iterate through all XML elements
            for action, elem in xml:
                # Skip comments and processing instructions,
                # because they do not have names
                if not (
                    isinstance(elem, etree._Comment)
                    or isinstance(elem, etree._ProcessingInstruction)
                ):
                    # Remove a namespace URI in the element's name
                    # elem.tag = etree.QName(elem).localname
                    if action == "end" and ":" in elem.tag:
                        elem.getparent().remove(elem)

            if xml.root is None:
                raise RuntimeError("XML root is empty")
            xml_root = xml.root

        # Remove comments
        etree.strip_tags(xml_root, etree.Comment)
        etree.cleanup_namespaces(xml_root)

        # Create and configure the URDF_visitor
        robot = Robot(name=xml_root.attrib["name"].__str__())  # Adjust as needed
        urdf_parser = URDF_parser(
            robot=robot,
            mesh_dir=kwargs.get("mesh_dir", ""),
            filename_handler=kwargs.get("filename_handler", None),
        )

        # Process the robot using Visitor pattern
        robot.visit(xml_root, urdf_parser)
        robot.init(kwargs.get("base_link_name", ""), kwargs.get("ee_link_name", ""), kwargs.get("merged_gripper_joint", False))

        return robot

    def visit_mimic(self, config: etree._Element | dict, element: Mimic):
        if not isinstance(config, etree._Element):
            return NotImplemented

        element.joint = config.get("joint")
        element.multiplier = str2float(config.get("multiplier", 1.0))
        element.offset = str2float(config.get("offset", 0.0))

    def visit_safety_controller(
        self, config: etree._Element | dict, element: SafetyController
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        element.soft_lower_limit = str2float(config.get("soft_lower_limit"))
        element.soft_upper_limit = str2float(config.get("soft_upper_limit"))
        element.k_position = str2float(config.get("k_position"))
        element.k_velocity = str2float(config.get("k_velocity"))

    def visit_box(self, config: etree._Element | dict, element: Box):
        if not isinstance(config, etree._Element):
            return NotImplemented
        # In case the element uses comma as a separator
        size = config.get("size", default="0 0 0").replace(",", " ")
        element.size = np.array(list(map(float, size.split())))

    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.radius = float(config.attrib["radius"])
        element.length = float(config.attrib["length"])

    def visit_sphere(self, config: etree._Element | dict, element: Sphere):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.radius = float(config.attrib["radius"])

    def visit_scale(self, config: etree._Element | dict, element: Scale):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.scale = None
        s = config.get("scale", default="").replace(",", " ").split()
        if s is not None:
            match len(s):
                case 0:
                    element.scale = None
                case 1:
                    element.scale = float(s[0])
                case _:
                    element.scale = np.array(list(map(float, s)))

    def visit_mesh(self, config: etree._Element | dict, element: Mesh):
        if not isinstance(config, etree._Element):
            return NotImplemented
        filename = config.get("filename")
        if filename:
            element.filename = filename

        self.visit_and_add_xml("scale", config, Scale, element)

    def visit_geometry(self, config: etree._Element | dict, element: Geometry):
        if not isinstance(config, etree._Element):
            return NotImplemented

        match config[0].tag:
            case "box":
                box = Box()
                box.visit(config[0], self)
                element.add(box)
            case "cylinder":
                cylinder = Cylinder()
                cylinder.visit(config[0], self)
                element.add(cylinder)
            case "mesh":
                mesh = Mesh()
                mesh.visit(config[0], self)
                element.add(mesh)
            case _:
                raise ValueError(f"Unknown tag: {config[0].tag}")

    def visit_origin(self, config: etree._Element | dict, element: Origin):
        if not isinstance(config, etree._Element):
            return NotImplemented
        xyz = config.get("xyz", default="0 0 0")
        rpy = config.get("rpy", default="0 0 0")

        element.xyz = np.array(list(map(float, xyz.split())))
        element.rpy = np.array(list(map(float, rpy.split())))

    def visit_color(self, config: etree._Element | dict, element: Color):
        if not isinstance(config, etree._Element):
            return NotImplemented
        rgba = config.get("rgba", default="1 1 1 1")
        element.rgba = np.array(list(map(float, rgba.split())))

    def visit_texture(self, config: etree._Element | dict, element: Texture):
        if not isinstance(config, etree._Element):
            return NotImplemented
        filename = config.get("filename", default=None)
        if filename:
            element.filename = filename

    def visit_material(self, config: etree._Element | dict, element: Material):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.name = config.get("name", "")

        self.visit_and_add_xml("color", config, Color, element)
        self.visit_and_add_xml("texture", config, Texture, element)

    def visit_visual(self, config: etree._Element | dict, element: Visual):
        if not isinstance(config, etree._Element):
            return NotImplemented
        name = config.get("name")
        if name:
            element.name = name

        self.visit_and_add_xml("origin", config, Origin, element)
        self.visit_and_add_xml("geometry", config, Geometry, element)
        self.visit_and_add_xml("material", config, Material, element)

    def visit_collision(self, config: etree._Element | dict, element: Collision):
        if not isinstance(config, etree._Element):
            return NotImplemented
        if "name" in config.attrib:
            element.name = config.attrib["name"].__str__()

        self.visit_and_add_xml("origin", config, Origin, element)
        self.visit_and_add_xml("geometry", config, Geometry, element)

    def visit_inertia(self, config: etree._Element | dict, element: Inertia):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.inertia_tensor = np.array(
            [
                [
                    config.get("ixx", default=1.0),
                    config.get("ixy", default=0.0),
                    config.get("ixz", default=0.0),
                ],
                [
                    config.get("ixy", default=0.0),
                    config.get("iyy", default=1.0),
                    config.get("iyz", default=0.0),
                ],
                [
                    config.get("ixz", default=0.0),
                    config.get("iyz", default=0.0),
                    config.get("izz", default=1.0),
                ],
            ],
            dtype=np.float64,
        )

    # def visit_mass(self, config: etree._Element | dict, element: Mass):
    #     if not isinstance(config, etree._Element):
    #         return NotImplemented
    #     element.value = str2float(config.get("value", default=0.0))

    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add_xml("origin", config, Origin, element)
        self.visit_and_add_xml("inertia", config, Inertia, element)
        # self.visit_and_add_xml("mass", config, Mass, element)
        element.mass = str2float(config.get("value", None))

    def visit_link(self, config: etree._Element | dict, element: Link):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add_xml("inertial", config, Inertial, element)

        for v in config.findall("visual"):
            visual = Visual()
            visual.visit(v, self)
            element.add(visual)

        for c in config.findall("collision"):
            collision = Collision()
            collision.visit(c, self)
            element.add(collision)

    def visit_axis(self, config: etree._Element | dict, element: Axis):
        if not isinstance(config, etree._Element):
            return NotImplemented
        xyz = config.get("xyz", "1 0 0")
        element.axis = np.array(list(map(float, xyz.split())))

    def visit_limit(self, config: etree._Element | dict, element: Limit):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.effort = str2float(config.get("effort", default=None))
        element.velocity = str2float(config.get("velocity", default=None))
        element.lower = str2float(config.get("lower", default=None))
        element.upper = str2float(config.get("upper", default=None))

    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.damping = str2float(config.get("damping", default=None))
        element.friction = str2float(config.get("friction", default=None))

    def visit_calibration(self, config: etree._Element | dict, element: Calibration):
        if not isinstance(config, etree._Element):
            return NotImplemented
        return super().visit_calibration(config, element)

    def visit_joint(self, config: etree._Element | dict, element: Joint):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.type = config.get("type", default=None)

        parent_xml = config.find("parent")
        if parent_xml is not None:
            parent_link_name = parent_xml.get("link")
            for l in self.robot.links:
                if l.name == parent_link_name:
                    element.parent = l.name
                    # element.add(l)
                    break

        child_xml = config.find("child")
        if child_xml is not None:
            child_link_name = child_xml.get("link")
            for l in self.robot.links:
                if l.name == child_link_name:
                    element.child = l.name
                    # element.add(l)
                    break

        self.visit_and_add_xml("origin", config, Origin, element)
        self.visit_and_add_xml("axis", config, Axis, element)
        self.visit_and_add_xml("limit", config, Limit, element)
        self.visit_and_add_xml("dynamics", config, Dynamics, element)
        self.visit_and_add_xml("mimic", config, Mimic, element)
        self.visit_and_add_xml("calibration", config, Calibration, element)
        self.visit_and_add_xml("safety_controller", config, SafetyController, element)

    def visit_robot(self, config: etree._Element | dict, element: Robot):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for link_xml in config.findall("link"):
            link = Link(name=link_xml.attrib["name"].__str__())
            link.visit(link_xml, self)
            element.add(link)

        for joint_xml in config.findall("joint"):
            joint = Joint(name=joint_xml.attrib["name"].__str__())
            joint.visit(joint_xml, self)
            element.add(joint)

        if config.find("gazebo") is not None:
            gazebo_visitor = GazeboUrdfParser()
            for gazebo_xml in config.findall("gazebo"):
                gazebo = Gazebo(reference=gazebo_xml.get("reference", None))
                gazebo.visit(gazebo_xml, gazebo_visitor)
                element.add(gazebo)

        if config.find("ros2_control") is not None:
            r2c_visitor = Ros2ControlUrdfParser()
            for r in config.findall("ros2_control"):
                r2c = Ros2Control(
                    name=r.attrib["name"].__str__(), type=r.attrib["type"].__str__()
                )

                r2c.visit(r, r2c_visitor)
                element.add(r2c)
