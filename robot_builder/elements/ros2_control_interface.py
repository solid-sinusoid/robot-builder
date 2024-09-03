from dataclasses import dataclass, field

from lxml import etree
from ..base.base import Visitor
from ..base.component import Component

@dataclass
class Param(Component):
    name: str
    value: str | None = None

    def visit(self, config, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_param(config, self)


@dataclass
class Hardware(Component):
    plugin: str | None = None
    params: list[Param] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_hardware(config, self)


@dataclass
class CommandInterface(Component):
    name: str
    params: list[Param] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_command_interface(config, self)


@dataclass
class StateInterface(Component):
    name: str
    params: list[Param] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_state_interface(config, self)


@dataclass
class Sensor(Component):
    name: str
    state_interface: list[StateInterface] = field(default_factory=list)
    params: list[Param] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_sensor(config, self)


@dataclass
class JointInterface(Component):
    name: str
    command_interface: list[CommandInterface] = field(default_factory=list)
    state_interface: list[StateInterface] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_joint_interface(config, self)


@dataclass
class Ros2Control(Component):
    name: str
    type: str | None = None
    hardware_interface: Hardware | None = None
    joint_interface: list[JointInterface] = field(default_factory=list)
    sensor: list[Sensor] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.ros2_control import Ros2ControlVisitor
        if not isinstance(visitor, Ros2ControlVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_ros2_control(config, self)
