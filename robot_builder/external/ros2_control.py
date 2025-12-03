from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from jinja2 import Environment, Template
import yaml
from robot_builder.logger import logger
from jinja2.runtime import Undefined

def yaml_str(value: Any) -> str:
    if value in (None, "", "~") or isinstance(value, Undefined):
        return '""'
    return str(value)


@dataclass
class ControllerManager:
    robot: Any
    cfg_path: Path
    var_map: Mapping[str, Any] = field(default_factory=dict)

    def _env(self) -> Environment:
        env = Environment(autoescape=False)
        env.filters["yaml_str"] = yaml_str
        return env

    def build(self) -> dict:
        context = self._compose_context()
        with open(self.cfg_path, encoding="utf-8") as f:
            template: Template = self._env().from_string(f.read())
        rendered = template.render(**context)
        return yaml.safe_load(rendered)

    def save(self, dst: Path) -> None:
        dst.parent.mkdir(parents=True, exist_ok=True)
        config = self.build()
        with open(dst, "w", encoding="utf-8") as f:
            yaml.dump(config, f, allow_unicode=True, sort_keys=False)
        logger.success("YAML with placeholders saved to → {}", dst)

    def _compose_context(self) -> dict[str, Any]:
        context = {
            name: getattr(self.robot, name)
            for name in dir(self.robot)
            if not name.startswith("_") and not callable(getattr(self.robot, name))
        }
        context.update(self.var_map)
        return context
