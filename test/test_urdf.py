import os
from robot_builder.parser.urdf import URDF_parser
from robot_builder.writer import URDF_writer

current_dir = os.path.dirname(__file__)
urdf_path = os.path.join(current_dir, "../urdf/current.urdf")
output_path = os.path.join(current_dir, "../urdf/test.urdf")

robot = URDF_parser.load(urdf_path)
print(f"Robot base link [ {robot.base_link} ]")
print(f"Robot ee_link [ {robot.ee_link} ]")
print(f"Robot actuated joint names [ {robot.actuated_joint_names} ]")
print(f"Robot gripper actuated joint names [ {robot.gripper_actuated_joint_names} ]")

# Записываем URDF
URDF_writer.write_xml_file(robot, output_path)
