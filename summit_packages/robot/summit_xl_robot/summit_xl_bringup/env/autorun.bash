#!/bin/bash

export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_HOSTNAME=$HOSTNAME

source ~/catkin_ws/devel/setup.bash

# SUMMIT XL
# AUTOBOOT
echo "ROBOTNIK SUMMIT XL v2.0"
Terminal=`tty`
case $Terminal in
    "/dev/tty3") sleep 10;
    screen -S core -d -m roscore;
    sleep 15;
    screen -S bringup -d -m rosrun rosmon rosmon --name=rosmon_bringup summit_xl_bringup robot_complete.launch;
    sleep 5;
    $ROBOT_RUN_MAP_NAV_MANAGER && screen -S map_nav_manager -d -m roslaunch summit_xl_bringup map_nav_manager.launch;
    sleep 5;
    $ROBOT_RUN_PERCEPTION && screen -S perception -d -m rosrun rosmon rosmon --name=rosmon_perception summit_xl_perception perception_complete.launch;
    sleep 5;
    $ROBOT_RUN_NAVIGATION && screen -S navigation -d -m rosrun rosmon rosmon --name=rosmon_navigation summit_xl_navigation navigation_complete.launch;
    sleep 2;
    $ROBOT_RUN_ROBOT_LOCAL_CONTROL && screen -S control -d -m rosrun rosmon rosmon --name=rosmon_robot_local_control summit_xl_robot_local_control robot_local_control.launch;
    sleep 5;
    screen -S robotnik_hmi -d -m rosrun rosmon rosmon --name=robotnik_hmi robotnik_hmi robotnik_hmi.launch;
    sleep 2;
    screen -S robotnik_rms -d -m rosrun rosmon rosmon --name=robotnik_rms robotnik_rms robotnik_rms.launch;
    sleep 2;
    $ROBOT_RUN_RLC_ROSTFUL_SERVER && screen -S rlc_rostful_server -d -m rosrun rosmon rosmon --name=rosmon_rlc_rostful_server summit_xl_robot_local_control rostful_server.launch;
    sleep 15;
    $ROBOT_HAS_ARM && screen -S arm_bringup -d -m rosrun rosmon rosmon --name=rosmon_arm summit_xl_bringup arm_complete.launch;;

esac
