from abc import ABC, abstractmethod
from typing import TYPE_CHECKING
from .base import Visitor

from lxml import etree

# if TYPE_CHECKING:
from ..elements.gazebo import (
    Gazebo,
    GzFts,
    GzMaterial,
    GzPbr,
    GzPbrMetal,
    GzPlugin,
    GzRos,
    GzSensor,
    GzVisual,
)


class GazeboVisitor(Visitor):
    # INFO: Gazebo part

    @abstractmethod
    def visit_gazebo(self, config: etree._Element | dict, element: Gazebo):
        pass

    @abstractmethod
    def visit_gz_visual(self, config: etree._Element | dict, element: GzVisual):
        pass

    @abstractmethod
    def visit_gz_material(self, config: etree._Element | dict, element: GzMaterial):
        pass

    @abstractmethod
    def visit_gz_sensor(self, config: etree._Element | dict, element: GzSensor):
        pass

    @abstractmethod
    def visit_gz_plugin(self, config: etree._Element | dict, element: GzPlugin):
        pass

    @abstractmethod
    def visit_gz_ros(self, config: etree._Element | dict, element: GzRos):
        pass

    @abstractmethod
    def visit_gz_fts(self, config: etree._Element | dict, element: GzFts):
        pass

    @abstractmethod
    def visit_gz_pbr_metal(self, config: etree._Element | dict, element: GzPbrMetal):
        pass

    @abstractmethod
    def visit_gz_pbr(self, config: etree._Element | dict, element: GzPbr):
        pass
