cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c16_decoder)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#message(STATUS "CMAKE: ${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        lslidar_c16_driver
        lslidar_c16_msgs
        dynamic_reconfigure
        pcl_conversions)

find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES lslidar_c16_decoder
        CATKIN_DEPENDS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        lslidar_c16_driver
        lslidar_c16_msgs
        dynamic_reconfigure
        pcl_conversions
        DEPENDS
        Boost
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIRS}

)


add_library(lslidar_data src/rawdata.cc)

target_link_libraries(lslidar_data
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        yaml-cpp)

add_library(lslidar_point src/convert.cc)
target_link_libraries(lslidar_point
        lslidar_data
        ${catkin_LIBRARIES})


add_library(lslidar_c16_decoder_nodelet src/cloud_nodelet.cc)
target_link_libraries(lslidar_c16_decoder_nodelet lslidar_point
        ${catkin_LIBRARIES})

add_executable(lslidar_c16_decoder_node src/cloud_node.cc)
target_link_libraries(lslidar_c16_decoder_node
        lslidar_point
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})
add_dependencies(lslidar_c16_decoder_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


if(catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_data ${catkin_EXPORTED_TARGETS})
endif()


install(TARGETS lslidar_data lslidar_point lslidar_c16_decoder_nodelet lslidar_c16_decoder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES
        nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

