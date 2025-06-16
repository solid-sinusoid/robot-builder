from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from jinja2 import Template
import yaml
from loguru import logger

@dataclass
class ControllerManager:
    """
    Generates YAML based on a Jinja2 template and variables from self.robot and var_map.
    """
    robot: Any
    cfg_path: Path
    var_map: Mapping[str, Any] = field(default_factory=dict)

    def build(self) -> dict:
        context = self._compose_context()
        with open(self.cfg_path) as f:
            template = Template(f.read())
        rendered = template.render(**context)
        return yaml.safe_load(rendered)

    def save(self, dst: Path) -> None:
        dst.parent.mkdir(parents=True, exist_ok=True)
        config = self.build()
        with open(dst, "w") as f:
            yaml.dump(config, f, allow_unicode=True, sort_keys=False)
        logger.success("YAML with placeholders saved to → {}", dst)

    def _compose_context(self) -> dict[str, Any]:
        context = {}
        for name in dir(self.robot):
            if not name.startswith("_"):
                attr = getattr(self.robot, name)
                if not callable(attr):
                    context[name] = attr
        context.update(self.var_map)
        return context
