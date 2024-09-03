from abc import ABC, abstractmethod
from .base import Visitor

from lxml import etree
from typing import TYPE_CHECKING

# if TYPE_CHECKING:
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


class RobotVisitor(Visitor):
    @abstractmethod
    def visit_origin(self, config: etree._Element | dict, element: Origin):
        pass

    @abstractmethod
    def visit_robot(self, config: etree._Element | dict, element: Robot):
        pass

    @abstractmethod
    def visit_joint(self, config: etree._Element | dict, element: Joint):
        pass

    @abstractmethod
    def visit_link(self, config: etree._Element | dict, element: Link):
        pass

    @abstractmethod
    def visit_scale(self, config: etree._Element | dict, element: Scale):
        pass

    @abstractmethod
    def visit_sphere(self, config: etree._Element | dict, element: Sphere):
        pass

    @abstractmethod
    def visit_cylinder(self, config: etree._Element | dict, element: Cylinder):
        pass

    @abstractmethod
    def visit_box(self, config: etree._Element | dict, element: Box):
        pass

    @abstractmethod
    def visit_mesh(self, config: etree._Element | dict, element: Mesh):
        pass

    @abstractmethod
    def visit_geometry(self, config: etree._Element | dict, element: Geometry):
        pass

    @abstractmethod
    def visit_color(self, config: etree._Element | dict, element: Color):
        pass

    @abstractmethod
    def visit_texture(self, config: etree._Element | dict, element: Texture):
        pass

    @abstractmethod
    def visit_material(self, config: etree._Element | dict, element: Material):
        pass

    @abstractmethod
    def visit_visual(self, config: etree._Element | dict, element: Visual):
        pass

    @abstractmethod
    def visit_collision(self, config: etree._Element | dict, element: Collision):
        pass

    @abstractmethod
    def visit_inertia(self, config: etree._Element | dict, element: Inertia):
        pass

    # @abstractmethod
    # def visit_mass(self, config: etree._Element | dict, element: Mass):
    #     pass

    @abstractmethod
    def visit_inertial(self, config: etree._Element | dict, element: Inertial):
        pass

    @abstractmethod
    def visit_axis(self, config: etree._Element | dict, element: Axis):
        pass

    @abstractmethod
    def visit_mimic(self, config: etree._Element | dict, element: Mimic):
        pass

    @abstractmethod
    def visit_safety_controller(
        self, config: etree._Element | dict, element: SafetyController
    ):
        pass

    @abstractmethod
    def visit_dynamics(self, config: etree._Element | dict, element: Dynamics):
        pass

    @abstractmethod
    def visit_limit(self, config: etree._Element | dict, element: Limit):
        pass

    @abstractmethod
    def visit_calibration(self, config: etree._Element | dict, element: Calibration):
        pass
