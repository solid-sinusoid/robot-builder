from dataclasses import field, dataclass

# from ..base.base import Visitor
from ..base.base import Component, Visitor
from typing import TYPE_CHECKING
# if TYPE_CHECKING:
#     from ..base.gazebo import GazeboVisitor

from typing import Annotated, Literal, TypeVar
import numpy as np
import numpy.typing as npt
from lxml import etree

DType = TypeVar("DType", bound=np.generic)
Array2 = Annotated[npt.NDArray[DType], Literal[2]]

@dataclass
class GzPbrMetal(Component):
    albedo_map: str | None = None
    normal_map: str | None = None
    ambient_occlusion_map: str | None = None
    roughness_map: str | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_pbr_metal(config, self)


@dataclass
class GzPbr(Component):
    metal: GzPbrMetal | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_pbr(config, self)


@dataclass
class GzMaterial(Component):
    diffuse: list[float] = field(default_factory=list)
    specular: list[float] = field(default_factory=list)
    ambient: list[float] = field(default_factory=list)
    lighting: bool | None = None
    emissive: list[float] = field(default_factory=list)
    pbr: GzPbr | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_material(config, self)


@dataclass
class GzVisual(Component):
    material: GzMaterial | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_visual(config, self)


@dataclass
class GzFts(Component):
    frame: str | None = None
    measure_direction: str | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_fts(config, self)

@dataclass
class GzCamera(Component):
    fov: float | None = None
    image_size: Array2[np.int16] | None = None
    clip: Array2[np.float32] | None = None


@dataclass
class GzSensor(Component):
    name: str
    type: str
    always_on: bool | None = None
    update_rate: int | None = None
    visualize: bool | None = None
    topic: str | None = None
    force_torque: GzFts | None = None
    camera: GzCamera | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_sensor(config, self)


@dataclass
class GzRos(Component):
    namespace: str | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_ros(config, self)


@dataclass
class GzPlugin(Component):
    filename: str | None = None
    name: str | None = None
    parameters: str | None = None
    ros: GzRos | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gz_plugin(config, self)


@dataclass
class Gazebo(Component):
    reference: str | None = None
    visual: GzVisual | None = None
    preserveFixedJoint: bool | None = None
    sensor: GzSensor | None = None
    plugin: GzPlugin | None = None
    self_collide: bool | None = None

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        from ..base.gazebo import GazeboVisitor
        if not isinstance(visitor, GazeboVisitor):
            raise ValueError("Type is not supported for this method")
        visitor.visit_gazebo(config, self)
