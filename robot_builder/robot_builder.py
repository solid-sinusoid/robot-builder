from abc import ABC, abstractmethod
from typing import Any, List

from robot_builder.urdf import Group, Joint, Link, Material

class RobotBuilderABC(ABC):
    @property
    @abstractmethod
    def robot(self) -> Any:
        pass

    @abstractmethod
    def base(self) -> None:
        pass

    @abstractmethod
    def gripper(self, gripper_package: str | None, save_geometry_path: str | None) -> None:
        pass

    @abstractmethod
    def ros2_control(self, hardware: str) -> None:
        pass

    @abstractmethod
    def moveit(self) -> None:
        pass
