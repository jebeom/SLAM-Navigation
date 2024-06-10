cmake_minimum_required(VERSION 3.0.2)
project(robotnik_laser_filters)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  filters
  laser_filters
  pluginlib
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/LinearInterpolationIntensityFilter.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotnik_laser_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(robotnik_laser_filters
  src/robotnik_laser_scan_filters.cpp
  src/linear_interpolation_intensity_filter.cpp
)

target_link_libraries(robotnik_laser_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(robotnik_laser_filters ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES robotnik_laser_filters_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotnik_laser_filters.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
