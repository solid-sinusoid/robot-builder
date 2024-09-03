from abc import ABC
from lxml import etree
from typing import Type
from dataclasses import fields

from .component import Component


class Visitor(ABC):
    def visit_and_add_xml(
        self,
        tag: str,
        config: etree._Element | dict,
        child: Type[Component],
        parent: Component,
    ):
        """
        Method that adds a `Component` found from the specified tag in the XML file.

        Args:
            tag: str
                A tag in XML
            config: etree._Element
                Current XML pointer
            child: Type[Component]
                The type of `Componet` to instantiate and add
            parent: Component
                The parent `Component` to which the new component will be added
        """
        if not isinstance(config, etree._Element):
            return NotImplemented
        component_xml = config.find(tag)
        if component_xml is not None:
            component = child()
            component.visit(component_xml, self)
            parent.add(component)

    def visit_and_add(
        self, config: etree._Element | dict, child: Type[Component], parent: Component
    ):
        if not isinstance(config, dict):
            return NotImplemented
        component = child()
        parent.add(component)
        component.visit(config, self)

    def _writing(self, config: etree._Element | dict, element: Component):
        """
        Method that visits all `Component` inside the current `Component`.

        It currently does not work with parsing Classes because this method
        does not support `None` values. However, if the classes are initialized,
        it works well.

        IMPORTANT: This also doesn't work with the Robot Component because this component has fields that are computed by a different Visitor.

        TODO: Rework this method for writing and parsing
        """
        if isinstance(config, dict):
            return NotImplemented
        for field in fields(element):
            value = getattr(element, field.name)
            if isinstance(value, Component):
                value.visit(config, self)

            elif isinstance(value, list) and value and isinstance(value[0], Component):
                for item in value:
                    if isinstance(item, Component):
                        item.visit(config, self)
                    else:
                        raise AttributeError(
                            f"List of Components should contain only Components, but found {type(item)}"
                        )
