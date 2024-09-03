from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, fields
from types import GenericAlias, UnionType
from typing import TYPE_CHECKING, get_args, get_origin, get_type_hints

from lxml import etree

if TYPE_CHECKING:
    from .base import Visitor


@dataclass(eq=False)
class Component(ABC):
    @property
    def _parent(self) -> "Component | None":
        return self._parent

    @_parent.setter
    def _parent(self, parent: "Component | None"):
        self.__parent = parent

    def add(self, component: "Component") -> None:
        hints = get_type_hints(self)
        for attr_name, attr_type in hints.items():
            origin = get_origin(attr_type)
            if origin is list:
                list_type = get_args(attr_type)[0]
                if isinstance(component, list_type):
                    getattr(self, attr_name).append(component)
                    component._parent = self
                    return
            elif isinstance(component, attr_type):
                setattr(self, attr_name, component)
                component._parent = self
                return
        raise ValueError(f"Type not supported: {type(component)}")

    def remove(self, component: "Component") -> None:
        for attr_name, attr_type in self.__annotations__.items():
            if (
                isinstance(component, attr_type)
                and getattr(self, attr_name) is component
            ):
                setattr(self, attr_name, None)
                component._parent = None
                return
        raise ValueError(
            f"Component not found or type not supported: {type(component)}"
        )

    # WARN: More testing is needed to confirm that this code works well.
    def __eq__(self, other):
        if not isinstance(other, Component):
            return NotImplemented
        if self.__annotations__ != other.__annotations__:
            return False
        for attr_name in self.__annotations__:
            if getattr(self, attr_name) != getattr(other, attr_name):
                return False
        return True

    @abstractmethod
    def visit(self, config: etree._Element | dict, visitor: Visitor):
        pass

    # INFO: Is this helpful or does it just reduce the speed of the code?
    def __post_init__(self):
        """
        Checks if the class is composite by determining if it has at least one field
        with the type `Component`. If such a field exists, the class is considered composite.

        This method performs the following steps:
        1. Retrieves all fields and their corresponding types in the class.
        2. Checks if the class is composite by identifying fields with the type `Component`
           or a `Union` type that includes `Component`.
        3. Sets the `_is_composite` attribute to `True` if a field of type `Component` is found.

        Attributes:
            _is_composite (bool): Indicates whether the class is composite.
        """
        data_class_fields = fields(self)
        type_hints = get_type_hints(self)

        self._is_composite = False

        for field in data_class_fields:
            field_name = field.name
            field_type = type_hints[field_name]

            # Check if the field type is a generic alias (e.g., List[Component])
            if isinstance(field_type, GenericAlias):
                self._is_composite = True
                return

            # Check if the field type is a Union that includes Component
            if isinstance(field_type, UnionType):
                args = get_args(field_type)
                for arg in args:
                    if issubclass(arg, Component):
                        self._is_composite = True
                        return
