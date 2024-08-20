
from dataclasses import asdict, fields
from lxml import etree
from robot_builder.base import *

class URDF_writer(Visitor):
    def __init__(
        self,
        filename_handler=None,
        mesh_dir: str = ""
    ) -> None:

        if filename_handler is None:
            self._filename_handler = partial(filename_handler_magic, dir=mesh_dir)
        else:
            self._filename_handler = filename_handler

    @staticmethod
    def write_xml(robot, **kwargs):
        """Write URDF model to an XML element hierarchy.

        Returns:
            etree.ElementTree: XML data.
        """
        config = URDF_writer.write(robot, *kwargs)
        return etree.ElementTree(config)

    @staticmethod
    def write_xml_string(robot, **kwargs):
        """Write URDF model to a string.

        Returns:
            str: String of the xml representation of the URDF model.
        """

        config = URDF_writer.write(robot, *kwargs)
        return etree.tostring(config, xml_declaration=True, *kwargs)

    @staticmethod
    def write_xml_file(robot, fname, **kwargs):
        """Write URDF model to an xml file.

        Args:
            fname (str): Filename of the file to be written. Usually ends in `.urdf`.
        """
        config = URDF_writer.write_xml(robot, *kwargs)
        config.write(fname, xml_declaration=True, pretty_print=True)

    @staticmethod
    def write(robot: Robot, **kwargs):
        config = etree.Element("robot", attrib={"name": robot.name})
        urdf_writer = URDF_writer()
        robot.visit(config, urdf_writer)
        return config

    def visit_robot(self, config: etree._Element | dict, element: Robot):
        if isinstance(config, dict):
            return NotImplemented
        self._writing(config, element)

    def visit_link(self, config: etree._Element | dict, element: Link):
        if isinstance(config, dict):
            return NotImplemented
        xml_child = etree.SubElement(config, "link", attrib={"name": element.name})
        self._writing(xml_child, element)


    def visit_joint(self, config: etree._Element | dict, element: Joint):
        if isinstance(config, dict):
            return NotImplemented
        if isinstance(config, dict):
            return NotImplemented
        xml_child = etree.SubElement(config, "joint")
        if element.type is not None:
            xml_child.attrib["type"] = element.type
        else:
            xml_child.attrib["type"] = ""

        xml_parent = etree.SubElement(xml_child, "parent")
        if element.parent is not None:
            xml_parent.attrib["link"] = element.parent
        else:
            xml_parent.attrib["link"] = ""
        
        xml_children = etree.SubElement(xml_child, "child")
        if element.child is not None:
            xml_children.attrib["link"] = element.child
        else:
            xml_children.attrib["link"] = ""

        self._writing(xml_child, element)

    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        if isinstance(config, dict):
            return NotImplemented
        xml_child = etree.SubElement(config, "inertial")
        self._writing(xml_child, element)

    def visit_inertia(self, config: etree._Element | dict, element: Inertia):
        if isinstance(config, dict):
            return NotImplemented
        if element.inertia_tensor is not None:
            etree.SubElement(config, "inertia", attrib={
                "ixx": str(element.inertia_tensor[0, 0]),
                "ixy": str(element.inertia_tensor[0, 1]),
                "ixz": str(element.inertia_tensor[0, 2]),
                "iyy": str(element.inertia_tensor[1, 1]),
                "iyz": str(element.inertia_tensor[1, 2]),
                "izz": str(element.inertia_tensor[2, 2]),
            })

    def visit_mass(self, config: etree._Element | dict, element: Mass):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "mass", attrib={"value": str(element.value)})

    def visit_axis(self, config: etree._Element | dict, element: Axis):
        if isinstance(config, dict):
            return NotImplemented
        if element.axis is not None:
            etree.SubElement(config, "axis", 
                             attrib={"xyz": " ".join(map(str, element.axis))})

    def visit_mimic(self, config: etree._Element | dict, element: Mimic):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "mimic", attrib={
            "joint": str(element.joint),
            "multiplier": str(element.multiplier),
            "offset": str(element.offset)
        })

    def visit_safety_controller(self, config: etree._Element | dict, element: SafetyController):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "safety_controller", attrib={
            "soft_lower_limit": str(element.soft_lower_limit),
            "soft_upper_limit": str(element.soft_upper_limit),
            "k_position": str(element.k_position),
            "k_velocity": str(element.k_velocity)
        })

    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {}
        if element.damping is not None:
            attrib["damping"] = str(element.damping)
        if element.friction is not None:
            attrib["friction"] = str(element.friction)

        etree.SubElement(config, "dynamics", attrib=attrib)

    def visit_limit(self, config: etree._Element | dict, element: Limit):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {}
        if element.effort is not None:
            attrib["effort"] = str(element.effort)
        if element.velocity is not None:
            attrib["velocity"] = str(element.velocity)
        if element.lower is not None:
            attrib["lower"] = str(element.lower)
        if element.upper is not None:
            attrib["upper"] = str(element.upper)

        etree.SubElement(config, "limit", attrib=attrib)

    def visit_calibration(self, config: etree._Element | dict, element: Calibration):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "calibration", attrib={
            "rising": str(element.rising),
            "falling": str(element.falling)
        })

    def visit_origin(self, config: etree._Element | dict, element: Origin):
        if isinstance(config, dict):
            return NotImplemented
        origin_attributes = {}
        
        if element.xyz is not None:
            origin_attributes["xyz"] = " ".join(map(str, element.xyz))
        
        if element.rpy is not None:
            origin_attributes["rpy"] = " ".join(map(str, element.rpy))
        
        etree.SubElement(config, "origin", attrib=origin_attributes)

    def visit_scale(self, config: etree._Element | dict, element: Scale):
        if isinstance(config, dict):
            return NotImplemented
        if element.scale is not None:
            if isinstance(element.scale, float) or isinstance(element.scale, int):
                config.set("scale", " ".join([str(element.scale)] * 3))
            else:
                config.set("scale", " ".join(map(str, element.scale)))


    def visit_sphere(self, config: etree._Element | dict, element: Sphere):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "sphere", attrib={
            "radius": str(element.radius)
        })

    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder):
        if isinstance(config, dict):
            return NotImplemented
        etree.SubElement(config, "cylinder", attrib={
            "radius": str(element.radius),
            "length": str(element.length)
        })

    def visit_box(self, config: etree._Element | dict, element: Box):
        if isinstance(config, dict):
            return NotImplemented
        if element.size is not None:
            etree.SubElement(config, "box", attrib={
                "size": " ".join(map(str, element.size))
            })

    def visit_mesh(self, config: etree._Element | dict, element: Mesh):
        if isinstance(config, dict):
            return NotImplemented
        xml_child = etree.SubElement(config, "mesh", attrib={
            "filename": self._filename_handler(element.filename)
        })
        self._writing(xml_child, element)

    def visit_geometry(self, config: etree._Element | dict, element: Geometry):
        if isinstance(config, dict):
            return NotImplemented
        xml_child = etree.SubElement(config, "geometry")
        self._writing(xml_child, element)

    def visit_color(self, config: etree._Element | dict, element: Color):
        if isinstance(config, dict):
            return NotImplemented
        if element.rgba is not None:
            etree.SubElement(config, "color", attrib={
                "rgba": " ".join(map(str, element.rgba))
            })

    def visit_texture(self, config: etree._Element | dict, element: Texture):
        if isinstance(config, dict):
            return NotImplemented
        if element.filename is not None:
            etree.SubElement(config, "texture", attrib={
                "filename": element.filename
            })

    def visit_material(self, config: etree._Element | dict, element: Material):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "material", attrib=attrib)

        self._writing(xml_child, element)

    def visit_visual(self, config: etree._Element | dict, element: Visual):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "visual", attrib=attrib)
        self._writing(xml_child, element)

    def visit_collision(self, config: etree._Element | dict, element: Collision):
        if isinstance(config, dict):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "collision", attrib=attrib)
        self._writing(xml_child, element)
        
    #INFO: ros2 control stuff
    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {}
        attrib["name"] = element.name if element.name is not None else ""
        attrib["type"] = element.type if element.type is not None else ""
        xml_child = etree.SubElement(config, "ros2_control", attrib=attrib)
        self._writing(xml_child, element)


    def visit_joint_interface(self, config: etree._Element | dict, element: JointInterface):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "joint", attrib=attrib)
        self._writing(xml_child, element)


    def visit_state_interface(self, config: etree._Element | dict, element: StateInterface):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "state_interface", attrib=attrib)
        self._writing(xml_child, element)


    def visit_command_interface(self, config: etree._Element | dict, element: CommandInterface):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "command_interface", attrib=attrib)
        self._writing(xml_child, element)


    def visit_sensor(self, config: etree._Element | dict, element: Sensor):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "sensor", attrib=attrib)
        self._writing(xml_child, element)



    def visit_hardware(self, config: etree._Element | dict, element: Hardware):
        if not isinstance(config, etree._Element):
            return NotImplemented
        hardware = etree.SubElement(config, "hardware")
        plugin = etree.SubElement(hardware, "plugin")
        plugin.text = element.plugin
        self._writing(hardware, element)

        

    def visit_param(self, config: etree._Element | dict, element: Param):
        if isinstance(config, dict):
            return NotImplemented
        param_xml = etree.SubElement(config, "param", attrib={"name": element.name})
        param_xml.text = element.value


    def _writing(self, config: etree._Element | dict | dict, element: Component):
        if isinstance(config, dict):
            return NotImplemented
        for field in fields(element):
            value = getattr(element, field.name)
            if isinstance(value, Component):
                value.visit(config, self)
            
            elif isinstance(value, list) and value and isinstance(value[0], Component):
                for item in value:
                    if isinstance(item, Component):
                        item.visit(config, self)
