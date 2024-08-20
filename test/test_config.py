
from robot_builder.parser import RobotConfigParser
from robot_builder.writer import URDF_writer

robot = RobotConfigParser.load("/home/bill-finger/rbs_ws/src/rbs_arm/config/robot_config.toml")

URDF_writer.write_xml_file(robot, "/home/bill-finger/rbs_ws/src/robot_builder/urdf/test_config.urdf")
