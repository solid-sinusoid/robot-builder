from abc import ABC, abstractmethod
from ..elements import Robot


class RobotBuilderABC(ABC):
    @property
    @abstractmethod
    def robot(self) -> Robot:
        pass

    @abstractmethod
    def base(self) -> None:
        pass

    @abstractmethod
    def gripper(
        self, gripper_package: str | None, save_geometry_path: str | None
    ) -> None:
        pass

    @abstractmethod
    def ros2_control(self, hardware: str) -> None:
        pass

    @abstractmethod
    def moveit(self) -> None:
        pass
