import os
from functools import partial
from typing import Type

import six
from loguru import logger
from lxml import etree

from robot_builder.base import Component, Visitor
from robot_builder.external.srdf_xml import SrdfRobot

from ..elements import *
from ..utils import filename_handler_magic


class SrdfParser(Visitor):
    def __init__(
        self,
        robot: SrdfRobot,
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
                    if action == "end" and ":" in elem.tag:  # pyright: ignore[]
                        elem.getparent().remove(elem)  # pyright: ignore[]

            if xml.root is None:
                raise RuntimeError("XML root is empty")
            xml_root = xml.root

        # Remove comments
        etree.strip_tags(xml_root, etree.Comment)  # pyright: ignore[]
        etree.cleanup_namespaces(xml_root)  # pyright: ignore[]
        # Create and configure the URDF_visitr

        robot = Robot(name=xml_root.attrib["name"])  # Adjust as needed
        urdf_parser = URDF_parser(
            robot=robot,
            mesh_dir=kwargs.get("mesh_dir", ""),
            filename_handler=kwargs.get("filename_handler", None),
        )

        # Process the robot using Visitor pattern
        robot.visit(xml_root, urdf_parser)
        robot.init()

        return robot

    def visit_mimic(self, config: etree._Element | dict, element: Mimic):
        pass

    def visit_safety_controller(
        self, config: etree._Element | dict, element: SafetyController
    ):
        pass

    def visit_box(self, config: etree._Element | dict, element: Box):
        pass

    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder):
        pass

    def visit_sphere(self, config: etree._Element | dict, element: Sphere):
        pass

    def visit_scale(self, config: etree._Element | dict, element: Scale):
        pass

    def visit_mesh(self, config: etree._Element | dict, element: Mesh):
        pass

    def visit_geometry(self, config: etree._Element | dict, element: Geometry):
        pass

    def visit_origin(self, config: etree._Element | dict, element: Origin):
        pass

    def visit_color(self, config: etree._Element | dict, element: Color):
        pass

    def visit_texture(self, config: etree._Element | dict, element: Texture):
        pass

    def visit_material(self, config: etree._Element | dict, element: Material):
        pass

    def visit_visual(self, config: etree._Element | dict, element: Visual):
        pass

    def visit_collision(self, config: etree._Element | dict, element: Collision):
        pass

    def visit_inertia(self, config: etree._Element | dict, element: Inertia):
        pass

    def visit_mass(self, config: etree._Element | dict, element: Mass):
        pass

    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        pass

    def visit_link(self, config: etree._Element | dict, element: Link):
        pass

    def visit_axis(self, config: etree._Element | dict, element: Axis):
        pass

    def visit_limit(self, config: etree._Element | dict, element: Limit):
        pass

    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics):
        pass

    def visit_calibration(self, config: etree._Element | dict, element: Calibration):
        pass

    def visit_joint(self, config: etree._Element | dict, element: Joint):
        pass

    def visit_robot(self, config: etree._Element | dict, element: Robot):
        pass

    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        pass

    def visit_joint_interface(
        self, config: etree._Element | dict, element: JointInterface
    ):
        pass

    def visit_state_interface(
        self, config: etree._Element | dict, element: StateInterface
    ):
        pass

    def visit_command_interface(
        self, config: etree._Element | dict, element: CommandInterface
    ):
        pass

    def visit_sensor(self, config: etree._Element | dict, element: Sensor):
        pass

    def visit_hardware(self, config: etree._Element | dict, element: Hardware):
        pass

    def visit_param(self, config: etree._Element | dict, element: Param):
        # if not isinstance(config, etree._Element):
        #     return NotImplemented
        pass

    def visit_gazebo(self, config: etree._Element | dict, element: Gazebo):
        pass

    def visit_gz_plugin(self, config: etree._Element | dict, element: GzPlugin):
        pass

    def visit_gz_sensor(self, config: etree._Element | dict, element: GzSensor):
        pass

    def visit_gz_fts(self, config: etree._Element | dict, element: GzFts):
        pass

    def visit_gz_visual(self, config: etree._Element | dict, element: GzVisual):
        pass

    def visit_gz_material(self, config: etree._Element | dict, element: GzMaterial):
        pass

    def visit_gz_pbr(self, config: etree._Element | dict, element: GzPbr):
        pass

    def visit_gz_pbr_metal(self, config: etree._Element | dict, element: GzPbrMetal):
        pass

    def visit_gz_ros(self, config: etree._Element | dict, element: GzRos):
        pass

    def visit_and_add(
        self,
        tag: str,
        config: etree._Element | dict,
        child: Type[Component],
        parent: Component,
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented
        component_xml = config.find(tag)
        if component_xml is not None:
            component = child()
            component.visit(component_xml, self)
            parent.add(component)
