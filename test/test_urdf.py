

from robot_builder.parser import URDF_parser
from robot_builder.writer import URDF_writer

robot = URDF_parser.load("/home/bill-finger/rbs_ws/src/robot_builder/urdf/current.urdf")
print(robot)

URDF_writer.write_xml_file(robot, "/home/bill-finger/rbs_ws/src/robot_builder/urdf/test.urdf")
