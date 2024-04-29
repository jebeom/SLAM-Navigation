from robot_simple_command_manager.parser.lists.base_lists import *

list_types = [StringList, IntList, FloatList, BoolList]

from robot_simple_command_manager.parser.lists.gps_coordinate_list import *

list_types.append(GPSCoordinateList)
