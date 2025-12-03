"""Centralized logging utilities for the robot_builder package."""

from __future__ import annotations

import os
import sys
from typing import Any

import logbook


def _default_level() -> int:
    """Resolve default log level from environment or fall back to INFO."""
    level_name = os.getenv("ROBOT_BUILDER_LOG_LEVEL", "INFO").upper()
    return getattr(logbook, level_name, logbook.INFO)


_handler = logbook.StreamHandler(sys.stdout, level=_default_level())
_handler.push_application()


class RobotBuilderLogger(logbook.Logger):
    """Logbook Logger with an extra success helper method."""

    def success(self, message: str, *args: Any, **kwargs: Any) -> None:
        self.log(logbook.NOTICE, message, *args, **kwargs)


logger = RobotBuilderLogger("robot_builder")


__all__ = ["logger", "RobotBuilderLogger"]
