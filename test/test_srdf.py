
from robot_builder.parser import SrdfParser

srdf = SrdfParser.load("/home/bill-finger/rbs_ws/src/robot_builder/urdf/test.srdf")
print(srdf)
