from lxml import etree

from ..base.ros2_control import Ros2ControlVisitor
from ..elements.ros2_control_interface import (
    CommandInterface,
    Hardware,
    JointInterface,
    Param,
    Ros2Control,
    Sensor,
    StateInterface,
)


class Ros2ControlWriter(Ros2ControlVisitor):
    def __init__(self) -> None:
        super().__init__()

    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {}
        attrib["name"] = element.name if element.name is not None else ""
        attrib["type"] = element.type if element.type is not None else ""
        xml_child = etree.SubElement(config, "ros2_control", attrib=attrib)
        self._writing(xml_child, element)

    def visit_joint_interface(
        self, config: etree._Element | dict, element: JointInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "joint", attrib=attrib)
        self._writing(xml_child, element)

    def visit_state_interface(
        self, config: etree._Element | dict, element: StateInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented
        attrib = {"name": element.name} if element.name is not None else {}
        xml_child = etree.SubElement(config, "state_interface", attrib=attrib)
        self._writing(xml_child, element)

    def visit_command_interface(
        self, config: etree._Element | dict, element: CommandInterface
    ):
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
