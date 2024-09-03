from ..base.ros2_control import Ros2ControlVisitor
from lxml import etree
from ..elements.ros2_control_interface import (
    CommandInterface,
    Hardware,
    JointInterface,
    Param,
    Ros2Control,
    Sensor,
    StateInterface,
)

class Ros2ControlUrdfParser(Ros2ControlVisitor):
    def __init__(self) -> None:
        super().__init__()

    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        if not isinstance(config, etree._Element):
            return NotImplemented
        self.visit_and_add_xml("hardware", config, Hardware, element)

        for j in config.findall("joint"):
            joint = JointInterface(name=j.attrib["name"].__str__())
            element.add(joint)
            joint.visit(j, self)

        for s in config.findall("sensor"):
            sensor = Sensor(name=s.attrib["name"].__str__())
            element.add(sensor)
            sensor.visit(s, self)

    def visit_joint_interface(
        self, config: etree._Element | dict, element: JointInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for si in config.findall("state_interface"):
            state_interface = StateInterface(name=si.attrib["name"].__str__())
            state_interface.visit(si, self)
            element.add(state_interface)

        for ci in config.findall("command_interface"):
            command_interace = CommandInterface(name=ci.attrib["name"].__str__())
            command_interace.visit(ci, self)
            element.add(command_interace)

    def visit_state_interface(
        self, config: etree._Element | dict, element: StateInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"].__str__(), value=p.text)
            element.add(param)

    def visit_command_interface(
        self, config: etree._Element | dict, element: CommandInterface
    ):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"].__str__(), value=p.text)
            element.add(param)

    def visit_sensor(self, config: etree._Element | dict, element: Sensor):
        if not isinstance(config, etree._Element):
            return NotImplemented

        for si in config.findall("state_interface"):
            state_interface = StateInterface(name=si.attrib["name"].__str__())
            state_interface.visit(si, self)
            element.add(state_interface)

    def visit_hardware(self, config: etree._Element | dict, element: Hardware):
        if not isinstance(config, etree._Element):
            return NotImplemented

        plugin = config.find("plugin")
        if plugin is not None:
            element.plugin = plugin.text

        for p in config.findall("param"):
            param = Param(name=p.attrib["name"].__str__(), value=p.text)
            element.add(param)

    def visit_param(self, config: etree._Element | dict, element: Param):
        # if not isinstance(config, etree._Element):
        #     return NotImplemented
        pass

