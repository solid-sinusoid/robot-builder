from abc import ABC, abstractmethod
from .base import Visitor

from lxml import etree
from typing import TYPE_CHECKING

# if TYPE_CHECKING:
from ..elements.ros2_control_interface import (
    CommandInterface,
    Hardware,
    JointInterface,
    Param,
    Ros2Control,
    Sensor,
    StateInterface,
)


class Ros2ControlVisitor(Visitor):
    # INFO: ros2_control part

    @abstractmethod
    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control):
        pass

    @abstractmethod
    def visit_joint_interface(
        self, config: etree._Element | dict, element: JointInterface
    ):
        pass

    @abstractmethod
    def visit_state_interface(
        self, config: etree._Element | dict, element: StateInterface
    ):
        pass

    @abstractmethod
    def visit_command_interface(
        self, config: etree._Element | dict, element: CommandInterface
    ):
        pass

    @abstractmethod
    def visit_sensor(self, config: etree._Element | dict, element: Sensor):
        pass

    @abstractmethod
    def visit_hardware(self, config: etree._Element | dict, element: Hardware):
        pass

    @abstractmethod
    def visit_param(self, config: etree._Element | dict, element: Param):
        pass
