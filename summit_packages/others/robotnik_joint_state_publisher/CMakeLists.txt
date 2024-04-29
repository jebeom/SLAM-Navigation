cmake_minimum_required(VERSION 3.0.2)
project(robotnik_joint_state_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rcomponent
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotnik_joint_state_publisher
  CATKIN_DEPENDS roscpp sensor_msgs rcomponent 
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# RobotnikJointStatePublisher
add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp src/JointStatePublisher.cpp)

add_dependencies(joint_state_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(joint_state_publisher_node
  ${catkin_LIBRARIES}
)

install(TARGETS joint_state_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
