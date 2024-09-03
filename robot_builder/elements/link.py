from dataclasses import dataclass, field
from typing import Annotated, Literal, TypeVar

import numpy as np
import numpy.typing as npt
from lxml import etree

from ..base.base import Visitor
from ..base.component import Component

DType = TypeVar("DType", bound=np.generic)
Array3 = Annotated[npt.NDArray[DType], Literal[3]]
Array4 = Annotated[npt.NDArray[DType], Literal[4]]


@dataclass
class Origin(Component):
    xyz: Array3[np.float32] | None = None
    rpy: Array3[np.float32] | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_origin(config, self)


@dataclass
class Scale(Component):
    scale: Array3[np.float32] | None | float = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_scale(config, self)


@dataclass
class Sphere(Component):
    radius: float | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_sphere(config, self)


@dataclass
class Cylinder(Component):
    radius: float | None = None
    length: float | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_cylinder(config, self)


@dataclass
class Box(Component):
    size: Array3[np.float32] | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_box(config, self)


@dataclass
class Mesh(Component):
    filename: str | None = None
    scale: Scale | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_mesh(config, self)


@dataclass
class Geometry(Component):
    box: Box | None = None
    cylinder: Cylinder | None = None
    sphere: Sphere | None = None
    mesh: Mesh | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_geometry(config, self)

    def is_composite(self) -> bool:
        return True


@dataclass
class Color(Component):
    """
    Color class

    Args:
        rgba: np.ndarray | None = None
            color description
    """
    rgba: Array4[np.float32] | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_color(config, self)


@dataclass
class Texture(Component):
    filename: str | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_texture(config, self)


@dataclass
class Material(Component):
    name: str | None = None
    color: Color | None = None
    texture: Texture | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_material(config, self)


@dataclass
class Visual(Component):
    name: str | None = None
    origin: Origin | None = None
    geometry: Geometry | None = None
    material: Material | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_visual(config, self)


@dataclass
class Collision(Component):
    name: str | None = None
    geometry: Geometry | None = None
    origin: Origin | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_collision(config, self)


@dataclass
class Inertia(Component):
    inertia_tensor: np.ndarray | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_inertia(config, self)


# @dataclass
# class Mass(Component):
#     value: float | None = None
#
#     def visit(self, config: etree._Element | dict, visitor: Visitor):
#         from ..base.robot import RobotVisitor
#         if not isinstance(visitor, RobotVisitor):
#             raise ValueError("Type is not supported by this method")
#         visitor.visit_mass(config, self)


@dataclass
class Inertial(Component):
    origin: Origin | None = None
    mass: float | None = None
    inertia: Inertia | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_inertial(config, self)


@dataclass
class Link(Component):
    name: str
    inertial: Inertial | None = None
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.robot import RobotVisitor
        if not isinstance(visitor, RobotVisitor):
            raise ValueError("Type is not supported by this method")
        visitor.visit_link(config, self)
