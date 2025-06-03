from lxml import etree
from robot_builder.base import Visitor, Component


class SrdfParserVisitor:
    def __init__(self) -> None:
        pass

    def visit_dataclass(self, config: etree._Element, element: Component):
        pass

    # def visit_and_add()


