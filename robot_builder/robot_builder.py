from abc import ABC, abstractmethod
from typing import Any

class RobotBuilderABC(ABC):
    @property
    @abstractmethod
    def robot(self) -> Any:
        pass

    @abstractmethod
    def base(self) -> None:
        pass

    @abstractmethod
    def gripper(self) -> None:
        pass

    @abstractmethod
    def ros2_control(self, hardware: str) -> None:
        pass

    @abstractmethod
    def moveit(self) -> None:
        pass
