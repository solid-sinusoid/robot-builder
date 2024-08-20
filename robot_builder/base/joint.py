from pydantic.dataclasses import dataclass
from lxml import etree
import numpy as np

from .link import Origin
# from .utils import array_eq, str2float
from .base import Component, Visitor, ComponentConfig

@dataclass(config=ComponentConfig.config)
class Axis(Component):
    axis: np.ndarray | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_axis(config, self)

@dataclass
class Mimic(Component):
    joint: str | None = None
    multiplier: float | None = None
    offset: float | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_mimic(config, self)

@dataclass
class SafetyController(Component):
    soft_lower_limit: float | None = None
    soft_upper_limit: float | None = None
    k_position: float | None = None
    k_velocity: float | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_safety_controller(config, self)


@dataclass
class Dynamics(Component):
    damping: float | None = None
    friction: float | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_dynamics(config, self)


@dataclass
class Limit(Component):
    effort: float | None = None
    velocity: float | None = None
    lower: float | None = None
    upper: float | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_limit(config, self)

@dataclass
class Calibration(Component):
    rising: float | None = None
    falling: float | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_calibration(config, self)


@dataclass
class GazeboReference(Component):
    name: str

    def visit(self, config, visitor: Visitor):
        return super().visit(config, visitor)

@dataclass
class Joint(Component):
    name: str
    type: str | None = None
    parent: str | None = None
    origin: Origin | None = None
    child: str | None = None
    axis: Axis | None = None
    dynamics: Dynamics | None = None
    limit: Limit | None = None
    mimic: Mimic | None = None
    calibration: Calibration | None = None
    safety_controller: SafetyController | None = None

    def visit(self, config, visitor: Visitor):
        visitor.visit_joint(config, self)
