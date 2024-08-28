import os
from functools import partial
from typing import Type

import numpy as np
import six
from loguru import logger
from lxml import etree

from ..base import Component, Visitor
from ..elements import *
from ..utils import filename_handler_magic, str2float


class URDF_parser(Visitor):
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
        element.filename = config.get("filename")

        self.visit_and_add("scale", config, Scale, element)

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
        element.filename = config.get("filename", default=None)

    def visit_material(self, config: etree._Element | dict, element: Material):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.name = config.get("name")

        self.visit_and_add("color", config, Color, element)
        self.visit_and_add("texture", config, Texture, element)

    def visit_visual(self, config: etree._Element | dict, element: Visual):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.name = config.get("name")

        self.visit_and_add("origin", config, Origin, element)
        self.visit_and_add("geometry", config, Geometry, element)
        self.visit_and_add("material", config, Material, element)

    def visit_collision(self, config: etree._Element | dict, element: Collision):
        if not isinstance(config, etree._Element):
            return NotImplemented
        if "name" in config.attrib:
            element.name = config.attrib["name"]

        self.visit_and_add("origin", config, Origin, element)
        self.visit_and_add("geometry", config, Geometry, element)

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

    def visit_mass(self, config: etree._Element | dict, element: Mass):
        if not isinstance(config, etree._Element):
            return NotImplemented
        element.value = str2float(config.get("value", default=0.0))

    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add("origin", config, Origin, element)
        self.visit_and_add("inertia", config, Inertia, element)
        self.visit_and_add("mass", config, Mass, element)

    def visit_link(self, config: etree._Element | dict, element: Link):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add("inertial", config, Inertial, element)

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

        self.visit_and_add("origin", config, Origin, element)
        self.visit_and_add("axis", config, Axis, element)
        self.visit_and_add("limit", config, Limit, element)
        self.visit_and_add("dynamics", config, Dynamics, element)
        self.visit_and_add("mimic", config, Mimic, element)
        self.visit_and_add("calibration", config, Calibration, element)
        self.visit_and_add("safety_controller", config, SafetyController, element)

    def visit_robot(self, config: etree._Element | dict, element: Robot):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for l in config.findall("link"):
            link = Link(name=l.attrib["name"])
            link.visit(l, self)
            element.add(link)

        for j in config.findall("joint"):
            joint = Joint(name=j.attrib["name"])
            joint.visit(j, self)
            element.add(joint)

        for g in config.findall("gazebo"):
            gazebo = Gazebo(reference=g.get("reference", None))
            gazebo.visit(g, self)
            element.add(gazebo)

        for r in config.findall("ros2_control"):
            r2c = Ros2Control(name=r.attrib["name"], type=r.attrib["type"])

            r2c.visit(r, self)
            element.add(r2c)

    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add("hardware", config, Hardware, element)

        for j in config.findall("joint"):
            joint = JointInterface(name=j.attrib["name"])
            element.add(joint)
            joint.visit(j, self)

        for s in config.findall("sensor"):
            sensor = Sensor(name=s.attrib["name"])
            element.add(sensor)
            sensor.visit(s, self)

    def visit_joint_interface(
        self, config: etree._Element | dict, element: JointInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for si in config.findall("state_interface"):
            state_interface = StateInterface(name=si.attrib["name"])
            state_interface.visit(si, self)
            element.add(state_interface)

        for ci in config.findall("command_interface"):
            command_interace = CommandInterface(name=ci.attrib["name"])
            command_interace.visit(ci, self)
            element.add(command_interace)

    def visit_state_interface(
        self, config: etree._Element | dict, element: StateInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"], value=p.text)
            element.add(param)

    def visit_command_interface(
        self, config: etree._Element | dict, element: CommandInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"], value=p.text)
            element.add(param)

    def visit_sensor(self, config: etree._Element | dict, element: Sensor):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for si in config.findall("state_interface"):
            state_interface = StateInterface(name=si.attrib["name"])
            state_interface.visit(si, self)
            element.add(state_interface)

    def visit_hardware(self, config: etree._Element | dict, element: Hardware):
        if not isinstance(config, etree._Element):
            return NotImplemented

        plugin = config.find("plugin")
        if plugin is not None:
            element.plugin = plugin.text

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"], value=p.text)
            element.add(param)

    def visit_param(self, config: etree._Element | dict, element: Param):
        # if not isinstance(config, etree._Element):
        #     return NotImplemented
        pass

    def visit_gazebo(self, config: etree._Element | dict, element: Gazebo):
        if isinstance(config, dict):
            return NotImplemented
        self.visit_and_add("visual", config, GzVisual, element)
        # self.visit_and_add("sensor", config, GzSensor, element)
        self.visit_and_add("plugin", config, GzPlugin, element)

        for s in config.findall("sensor"):
            s_name = s.get("name")
            s_type = s.get("type")
            if s_name and s_type:
                sensor = GzSensor(name=s_name, type=s_type)
                sensor.visit(s, self)
                element.add(sensor)

        pFJ = config.find("preserveFixedJoint")
        if pFJ is not None:
            element.preserveFixedJoint = bool(pFJ.text)

        self_collide = config.find("self_collide")
        if self_collide is not None:
            element.self_collide = bool(self_collide.text)

    def visit_gz_plugin(self, config: etree._Element | dict, element: GzPlugin):
        if isinstance(config, dict):
            return NotImplemented

        element.filename = config.get("filename", None)
        element.name = config.get("name", None)

        for p in config.findall("parameters"):
            element.parameters = p.text

        self.visit_and_add("ros", config, GzRos, element)

    def visit_gz_sensor(self, config: etree._Element | dict, element: GzSensor):
        if isinstance(config, dict):
            return NotImplemented

        always_on = config.find("always_on")
        if always_on is not None:
            element.always_on = bool(always_on.text)

        update_rate = config.find("update_rate")
        if update_rate is not None:
            if update_rate.text is not None:
                element.update_rate = int(update_rate.text)

        visualize = config.find("visualize")
        if visualize is not None:
            element.visualize = bool(visualize.text)

        topic = config.find("topic")
        if topic is not None:
            element.topic = topic.text

        self.visit_and_add("force_torque", config, GzFts, element)

    def visit_gz_fts(self, config: etree._Element | dict, element: GzFts):
        if isinstance(config, dict):
            return NotImplemented

        frame = config.find("frame")
        if frame is not None:
            element.frame = frame.text

        measure_direction = config.find("measure_direction")
        if measure_direction is not None:
            element.measure_direction = measure_direction.text

    def visit_gz_visual(self, config: etree._Element | dict, element: GzVisual):
        if isinstance(config, dict):
            return NotImplemented

        self.visit_and_add("material", config, GzMaterial, element)

    def visit_gz_material(self, config: etree._Element | dict, element: GzMaterial):
        if isinstance(config, dict):
            return NotImplemented

        diffuse = config.findtext("diffuse")
        if diffuse is not None:
            element.diffuse = [float(i) for i in diffuse.split(" ")]

        specular = config.findtext("specular")
        if specular is not None:
            element.specular = [float(i) for i in specular.split(" ")]

        ambient = config.findtext("ambient")
        if ambient is not None:
            element.ambient = [float(i) for i in ambient.split(" ")]

        emissive = config.findtext("emissive")
        if emissive is not None:
            element.emissive = [float(i) for i in emissive.split(" ")]

        lighting = config.findtext("lighting")
        if lighting is not None:
            element.lighting = bool(lighting)

        self.visit_and_add("pbr", config, GzPbr, element)

    def visit_gz_pbr(self, config: etree._Element | dict, element: GzPbr):
        if isinstance(config, dict):
            return NotImplemented

        self.visit_and_add("metal", config, GzPbrMetal, element)

    def visit_gz_pbr_metal(self, config: etree._Element | dict, element: GzPbrMetal):
        if isinstance(config, dict):
            return NotImplemented

        albedo = config.findtext("albedo_map")
        if albedo is not None:
            element.albedo_map = albedo

        normal = config.findtext("normal_map")
        if normal is not None:
            element.normal_map = normal

        ambient_occlusion = config.findtext("ambient_occlusion_map")
        if ambient_occlusion is not None:
            element.ambient_occlusion_map = ambient_occlusion

        roughness = config.findtext("roughness_map")
        if roughness is not None:
            element.roughness_map = roughness

    def visit_gz_ros(self, config: etree._Element | dict, element: GzRos):
        if isinstance(config, dict):
            return NotImplemented

        namespace = config.findtext("namespace")
        if namespace is not None:
            element.namespace = namespace

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
