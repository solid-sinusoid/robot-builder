from __future__ import annotations
from abc import ABC, abstractmethod
# from dataclasses import dataclass
from pydantic.dataclasses import dataclass
from pydantic import ConfigDict

from typing import TYPE_CHECKING, get_type_hints, get_origin, get_args

if TYPE_CHECKING:
    from .link import *
    from .robot import *
    from .joint import *
    from .ros2_control_interface import *



class Visitor(ABC):

    @abstractmethod
    def visit_origin(self, config: etree._Element | dict, element: Origin): pass

    @abstractmethod
    def visit_robot(self, config: etree._Element | dict, element: Robot): pass

    @abstractmethod
    def visit_joint(self, config: etree._Element | dict, element: Joint): pass

    @abstractmethod
    def visit_link(self, config: etree._Element | dict, element: Link): pass

    @abstractmethod
    def visit_scale(self, config: etree._Element | dict, element: Scale): pass

    @abstractmethod
    def visit_sphere(self, config: etree._Element | dict, element: Sphere): pass

    @abstractmethod
    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder): pass

    @abstractmethod
    def visit_box(self, config: etree._Element | dict, element: Box): pass

    @abstractmethod
    def visit_mesh(self, config: etree._Element | dict, element: Mesh): pass

    @abstractmethod
    def visit_geometry(self, config: etree._Element | dict, element: Geometry): pass

    @abstractmethod
    def visit_color(self, config: etree._Element | dict, element: Color): pass

    @abstractmethod
    def visit_texture(self, config: etree._Element | dict, element: Texture): pass

    @abstractmethod
    def visit_material(self, config: etree._Element | dict, element: Material): pass

    @abstractmethod
    def visit_visual(self, config: etree._Element | dict, element: Visual): pass

    @abstractmethod
    def visit_collision(self, config: etree._Element | dict, element: Collision): pass

    @abstractmethod
    def visit_inertia(self, config: etree._Element | dict, element: Inertia): pass

    @abstractmethod
    def visit_mass(self, config: etree._Element | dict, element: Mass): pass

    @abstractmethod
    def visit_inertial(self, config: etree._Element | dict, element: Inertial): pass

    @abstractmethod
    def visit_axis(self, config: etree._Element | dict, element: Axis): pass

    @abstractmethod
    def visit_mimic(self, config: etree._Element | dict, element: Mimic): pass

    @abstractmethod
    def visit_safety_controller(self, config: etree._Element | dict, element: SafetyController): pass

    @abstractmethod
    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics): pass

    @abstractmethod
    def visit_limit(self, config: etree._Element | dict, element: Limit): pass

    @abstractmethod
    def visit_calibration(self, config: etree._Element | dict, element: Calibration): pass

    #INFO: ros2_control part

    @abstractmethod
    def visit_ros2_control(self, config: etree._Element | dict, element: Ros2Control): pass

    @abstractmethod
    def visit_joint_interface(self, config: etree._Element | dict, element: JointInterface): pass

    @abstractmethod
    def visit_state_interface(self, config: etree._Element | dict, element: StateInterface): pass

    @abstractmethod
    def visit_command_interface(self, config: etree._Element | dict, element: CommandInterface): pass

    @abstractmethod
    def visit_sensor(self, config: etree._Element | dict, element: Sensor): pass

    @abstractmethod
    def visit_hardware(self, config: etree._Element | dict, element: Hardware): pass

    @abstractmethod
    def visit_param(self, config: etree._Element | dict, element: Param): pass



class ComponentConfig:
    config = ConfigDict(arbitrary_types_allowed=True)

@dataclass(eq=False)
class Component(ComponentConfig, ABC):

    @property
    def _parent(self) -> 'Component | None':
        return self._parent

    @_parent.setter
    def _parent(self, parent: 'Component | None'):
        self.__parent = parent

    def add(self, component: 'Component') -> None:
        hints = get_type_hints(self)
        for attr_name, attr_type in hints.items():
            origin = get_origin(attr_type)
            if origin is list:
                list_type = get_args(attr_type)[0]
                if isinstance(component, list_type):
                    getattr(self, attr_name).append(component)
                    component._parent = self
                    return
            elif isinstance(component, list):
                print(type(component))
            elif isinstance(component, attr_type):
                setattr(self, attr_name, component)
                component._parent = self
                return
        raise ValueError(f"Type not supported: {type(component)}")

    def remove(self, component: 'Component') -> None:
        for attr_name, attr_type in self.__annotations__.items():
            if isinstance(component, attr_type) and getattr(self, attr_name) is component:
                setattr(self, attr_name, None)
                component._parent = None
                return
        raise ValueError(f"Component not found or type not supported: {type(component)}")

    #WARN: this code is under development
    def __eq__(self, other):
        if not isinstance(other, Component):
            return NotImplemented
        if self.__annotations__ != other.__annotations__:
            return False
        for attr_name in self.__annotations__:
            if getattr(self, attr_name) != getattr(other, attr_name):
                return False
        return True


    @abstractmethod
    def visit(self, config: etree._Element | dict, visitor: Visitor):
        pass
