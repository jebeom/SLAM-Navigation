#!/bin/bash

# Run this script with the workspace compiled with catkin build and sourced
# It will create the new version inside workspace folder
# WARNING: For using catkin build, you need to install with apt the python-catkin-tools

# INSTALL DEPENDENCIES
sudo apt install python-bloom dpkg-dev debhelper


# LOAD PARAMETERS

if [ -z "$ROBOT_MODEL" ]
then
    echo -e "\n\e[1mThe parameter \$ROBOT_MODEL is not defined. Write the model of the robot:"
    read robot_name
else
    robot_name=$ROBOT_MODEL
fi

ws_path=`echo $CMAKE_PREFIX_PATH | awk '{split($1, a, "/devel:"); print a[1];}'`
packages_folder=$robot_name\_packages
ws_packages_path=$ws_path/src/$packages_folder

# Check if $ws_packages_path exist
[ -d "$ws_packages_path" ] && echo -e "\n\e[1mThe folder $packages_folder has been found correctly."
[ ! -d "$ws_packages_path" ] && echo -e "\n\e[1mThe folder $packages_folder has not been found." && exit -1

ws_version=`awk 'NR==1 {print substr($2,2);}' $ws_packages_path/CHANGELOG.rst`
ros_distro=$ROS_DISTRO
ws_name=$robot_name\_src_$ros_distro\_$ws_version


# CREATE .DEB FILES

# Set current working directory
cd $ws_packages_path

# .deb creation
sudo python $ws_packages_path/deb/build_system/rosdep_creator.py --ros-distro $ros_distro
rosdep update
python $ws_packages_path/deb/build_system/builder.py --ros-distro $ros_distro


# CLEAN AND GENERATE VERSION

echo -e "\n\e[1m¿Do you want to remove the private packages of the current workspace and create a new version? (yes/NO):"
read generate_version

if [ $generate_version = "yes" ]
then
    # remove private folder and git files
    rm -rf private/ .git/ .gitmodules

    # compress workspace
    cd $ws_path
    tar -czvf $ws_name.tar.gz src

    echo -e "\n\e[1m\e[92mThe new version has been generated successfully.\n"
else
    echo -e "\n\e[1m\e[91mAborted the generation of the new version.\n"
fi