cmake_minimum_required(VERSION 3.0.2)
project(ur_gripper_hw_interface)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    hardware_interface
    roscpp
    std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ur_gripper_hw_interface
   CATKIN_DEPENDS controller_manager hardware_interface roscpp std_msgs
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/ur_gripper_hw_interface.cpp
)

add_executable(ur_gripper_hw_interface_node src/ur_gripper_hw_interface_node.cpp)
add_dependencies(ur_gripper_hw_interface_node ${PROJECT_NAME})
target_link_libraries(ur_gripper_hw_interface_node
    ${PROJECT_NAME}
	${catkin_LIBRARIES})
