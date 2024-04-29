cmake_minimum_required(VERSION 3.0.2)
project(summit_xl_advanced_sim)

find_package(catkin REQUIRED)

catkin_package(
)

#############
## Install ##
#############
install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
