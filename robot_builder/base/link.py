from dataclasses import field
from pydantic.dataclasses import dataclass
from pydantic import ConfigDict
from lxml import etree
import numpy as np
from .base import Component, Visitor, ComponentConfig


from typing import Annotated, Literal, TypeVar
import numpy.typing as npt

DType = TypeVar("DType", bound=np.generic)
Array3 = Annotated[npt.NDArray[DType], Literal[3]]

@dataclass(config=ComponentConfig.config)
class Origin(Component):
    xyz: Array3[np.float32] | None = None
    rpy: Array3[np.float32] | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_origin(config, self)

    # def build(self, config: dict, visitor: Visitor):
    #     positon = config.get("position", {})
    #     if positon:
    #         pos = []
    #         for key in positon:
    #             pos.append(positon[key])
    #         self.xyz = np.array(pos)
    #     orientation = config.get("orientation", {})
    #     if orientation:
    #         orient = []
    #         for key in orientation:
    #             orient.append(orientation[key])
    #         self.rpy = np.array(orient)


@dataclass(config=ComponentConfig.config)
class Scale(Component):
    scale: Array3[np.float32] | None | float = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_scale(config, self)

@dataclass
class Sphere(Component):
    radius: float

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_sphere(config, self)

@dataclass
class Cylinder(Component):
    radius: float | None = None
    length: float | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_cylinder(config, self)

@dataclass(config=ComponentConfig.config)
class Box(Component):
    size: Array3[np.float32] | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_box(config, self)

@dataclass
class Mesh(Component):
    filename: str | None = None
    scale: Scale | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_mesh(config, self)


@dataclass
class Geometry(Component):
    box: Box | None = None
    cylinder: Cylinder | None = None
    sphere: Sphere | None = None
    mesh: Mesh | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_geometry(config, self)

    def is_composite(self) -> bool:
        return True

@dataclass(config=ComponentConfig.config)
class Color(Component):
    rgba: np.ndarray | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_color(config, self)

@dataclass
class Texture(Component):
    filename: str | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_texture(config, self)


@dataclass
class Material(Component):
    name: str | None = None
    color: Color | None = None
    texture: Texture | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_material(config, self)

@dataclass
class Visual(Component):
    name: str | None = None
    origin: Origin | None = None
    geometry: Geometry | None = None
    material: Material | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_visual(config, self)

@dataclass
class Collision(Component):
    name: str | None = None
    geometry: Geometry | None = None
    origin: Origin | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_collision(config, self)

    def is_composite(self) -> bool:
        return True

@dataclass(config=ComponentConfig.config)
class Inertia(Component):
    inertia_tensor: np.ndarray | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_inertia(config, self)


@dataclass
class Mass(Component):
    value: float | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_mass(config, self)


@dataclass
class Inertial(Component):
    origin: Origin | None = None
    mass: Mass | None = None
    inertia: Inertia | None = None 

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_inertial(config, self)

@dataclass
class Link(Component):
    name: str
    inertial: Inertial | None = None
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_link(config, self)

