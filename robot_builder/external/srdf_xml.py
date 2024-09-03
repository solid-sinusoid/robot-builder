from dataclasses import dataclass, field

from lxml import etree

from ..base.component import Component, VisitorType


@dataclass
class Sphere(Component):
    center: list[float]  # 3 space-separated float values (x, y, z)
    radius: float  # A floating-point value

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "sphere"


@dataclass
class LinkSphereApproximation(Component):
    link: str  # Name of the link that is approximated by spheres
    spheres: list[Sphere] = field(default_factory=list)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)


@dataclass
class PassiveJoint(Component):
    name: str  # Name of the passive joint

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "passive_joint"


@dataclass
class DisableCollisions(Component):
    link1: str  # Name of the first link in the pair
    link2: str  # Name of the second link in the pair
    reason: str | None = None  # Optional reason for disabling collisions

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "disable_collision"


@dataclass
class VirtualJoint(Component):
    name: str  # Name of the joint
    child_link: str  # Name of the link connecting the robot to the environment
    parent_frame: str  # Name of the frame assumed to be fixed
    type: str  # Type of joint (fixed, floating, planar)

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "virtual_joint"


@dataclass
class EndEffector(Component):
    name: str  # Name of the end effector
    group: str  # Name of the group containing links and joints
    parent_link: str  # Name of the link this end effector is attached to
    parent_group: str | None = (
        None  # Optional name of the joint group containing parent_link
    )

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)


@dataclass
class Chain(Component):
    base_link: str  # Root link of the chain
    tip_link: str  # Last link of the chain

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "chain"


@dataclass
class JointGroup(Component):
    name: str  # Name of the joint
    value: list[float] = field(
        default_factory=list
    )  # Value attribute in the <group_state> tag

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "joint"


@dataclass
class LinkGroup(Component):
    name: str  # Name of the link

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "link"


@dataclass
class GroupState(Component):
    name: str  # Name of the state
    group: str  # Name of the group the state is for
    joints: list[JointGroup] = field(default_factory=list)  # List of joints with values

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "group_state"


@dataclass
class Group(Component):
    name: str  # Name of the group
    links: list[LinkGroup] = field(default_factory=list)  # List of links in the group
    joints: list[JointGroup] = field(
        default_factory=list
    )  # List of joints in the group
    chains: list[Chain] = field(default_factory=list)  # List of chains in the group
    sub_groups: list["Group"] = field(default_factory=list)  # List of sub-groups

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "group"


@dataclass
class SrdfRobot(Component):
    name: str  # Name of the robot
    groups: list[Group] = field(default_factory=list)  # List of groups
    group_states: list[GroupState] = field(default_factory=list)  # List of group states
    end_effectors: list[EndEffector] = field(
        default_factory=list
    )  # List of end effectors
    virtual_joints: list[VirtualJoint] = field(
        default_factory=list
    )  # List of virtual joints
    disable_collisions: list[DisableCollisions] = field(
        default_factory=list
    )  # List of disable_collisions tags
    passive_joints: list[PassiveJoint] = field(
        default_factory=list
    )  # List of passive joints
    link_sphere_approximations: list[LinkSphereApproximation] = field(
        default_factory=list
    )  # List of link sphere approximations

    def visit(self, config: etree._Element | dict, visitor: Visitor):
        visitor.visit_dataclass(config, self)

    @property
    def alias(self) -> str:
        return "robot"
