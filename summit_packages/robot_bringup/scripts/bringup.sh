#!/bin/bash

# function to echo with color red [echo_error]
function echo_error() {
  echo -e "\e[31m$1\e[0m"
}
# function to echo with color green [echo_success]
function echo_success() {
  echo -e "\e[32m$1\e[0m"
}
# function to echo with color yellow [echo_warning]
function echo_warning() {
  echo -e "\e[33m$1\e[0m"
}
# function to echo with color blue and bold [echo_info]
function echo_info() {
  echo -e "\e[1;34m$1\e[0m"
}

ros_config="/home/robot/ros_config.sh"




# Loads $ros_config file if exists.
if [ -f "$ros_config" ]; then
  echo_success "Loading ros_config from $ros_config"
  if [ "$?" != "0" ]; then
    echo_error "Error reading file!! $ros_config"
    exit
  fi
else
# Loads ros_config using the path of the current script
  echo_warning "$ros_config doesn't exist. Loading config from robot_bringup/scripts/ros_config.sh"
  ros_config="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros_config.sh"
  if [ -f "$ros_config" ]; then
    echo_success "Loading ros_config from $ros_config"
    if [ "$?" != "0" ]; then
      echo "Error reading file!! $ros_config"
      exit
    fi
  fi
fi

if ! source $ros_config; then
  echo_error "Error reading ros_config.sh"
  exit
else
  echo_success "Loaded ros_config"
fi

echo_success "This script is going to load the ros configuration located at $ros_config"
echo "This script is going to load and launch all the robot packages based on the environment variables configuration..."
sleep 2;

# TODO: check if env vars is loaded and show a warning

echo "Closing all the running screens..."
killall screen
sleep 2;

echo "Starting roscore..."
screen -S core -d -m roscore;
sleep 2;

if [[ -z "${ROBOT_RUN_SIMULATION}" ]]; then
  ROBOT_RUN_SIMULATION=false
fi
if [[ -z "${ROBOT_RUN_LOCALIZATION}" ]]; then
  ROBOT_RUN_LOCALIZATION=false
fi
if [[ -z "${ROBOT_RUN_NAVIGATION}" ]]; then
  ROBOT_RUN_NAVIGATION=false
fi
if [[ -z "${ROBOT_RUN_PERCEPTION}" ]]; then
  ROBOT_RUN_PERCEPTION=false
fi
if [[ -z "${ROBOT_RUN_ROBOT_LOCAL_CONTROL}" ]]; then
  ROBOT_RUN_ROBOT_LOCAL_CONTROL=false
fi
if [[ -z "${ROBOT_RUN_RLC_ROSTFUL_SERVER}" ]]; then
  ROBOT_RUN_RLC_ROSTFUL_SERVER=false
fi
if [[ -z "${ROBOT_RUN_HMI}" ]]; then
  ROBOT_RUN_HMI=false
fi
if [[ -z "${ROBOT_RUN_COMMAND_MANAGER}" ]]; then
  ROBOT_RUN_COMMAND_MANAGER=false
fi
if [[ -z "${ROBOT_RUN_SENSORS}" ]]; then
  ROBOT_RUN_SENSORS=false
fi
if [[ -z "${ROBOT_RUN_ARM}" ]]; then
  ROBOT_RUN_ARM=false
fi
if [[ -z "${ROBOT_RUN_NEW_HMI}" ]]; then
  ROBOT_RUN_NEW_HMI=false
fi
if [[ -z "${ROBOT_RUN_ROSBAG_MANAGER}" ]]; then
  ROBOT_RUN_ROSBAG_MANAGER=false
fi
if [[ -z "${ROBOT_RUN_INTERFACES}" ]]; then
  ROBOT_RUN_INTERFACES=false
fi
if [[ -z "${ROBOT_RUN_SAFETY}" ]]; then
  ROBOT_RUN_SAFETY=false
fi
if [[ -z "${ROBOT_RUN_EXTERNAL_MACHINE}" ]]; then
  ROBOT_RUN_EXTERNAL_MACHINE=false
fi

echo "RUN SIM = $ROBOT_RUN_SIMULATION"
echo "RUN LOCALIZATION = $ROBOT_RUN_LOCALIZATION"
echo "RUN NAVIGATION = $ROBOT_RUN_NAVIGATION"
echo "RUN PERCEPTION = $ROBOT_RUN_PERCEPTION"
echo "RUN RLC = $ROBOT_RUN_ROBOT_LOCAL_CONTROL"
echo "RUN HMI = $ROBOT_RUN_HMI"
echo "RUN COMMAND_MANAGER = $ROBOT_RUN_COMMAND_MANAGER"
echo "RUN ROBOT_RUN_SENSORS = $ROBOT_RUN_SENSORS"
echo "RUN ROBOT_RUN_ARM = $ROBOT_RUN_ARM"
echo "RUN ROBOT_RUN_NTRIP = $ROBOT_RUN_NTRIP"
echo "RUN ROBOT_RUN_ROSBAG_MANAGER = $ROBOT_RUN_ROSBAG_MANAGER"
echo "RUN ROBOT_RUN_INTERFACES = $ROBOT_RUN_INTERFACES"
echo "RUN ROBOT_RUN_SAFETY = $ROBOT_RUN_SAFETY"
echo "RUN ROBOT_RUN_EXTERNAL_MACHINE = $ROBOT_RUN_EXTERNAL_MACHINE"

sleep 2;

if $ROBOT_RUN_SIMULATION
then
  if [[ -z "${ROBOT_BRINGUP_SIM_PACKAGE}" ]]; then
    echo_error "-> ERROR. The var ROBOT_BRINGUP_SIM_PACKAGE is not defined..."
    exit
  fi
  if [[ -z "${ROBOT_BRINGUP_SIM_LAUNCH}" ]]; then
    echo_error "-> ERROR. The var ROBOT_BRINGUP_SIM_LAUNCH is not defined..."
    exit
  fi
  echo_success "-> Launching robot simulation..." 
  echo_info "--> robot xacro: $ROBOT_XACRO ; path: $ROBOT_BRINGUP_SIM_PACKAGE/$ROBOT_BRINGUP_SIM_LAUNCH"
  screen -S bringup -d -m rosrun rosmon rosmon --name=rosmon_bringup $ROBOT_BRINGUP_SIM_PACKAGE $ROBOT_BRINGUP_SIM_LAUNCH;
else
  if [[ -z "${ROBOT_BRINGUP_PACKAGE}" ]]; then
    echo_error "-> ERROR. The var ROBOT_BRINGUP_PACKAGE is not defined..."
    exit
  fi
  if [[ -z "${ROBOT_BRINGUP_LAUNCH}" ]]; then
    echo_error "-> ERROR. The var ROBOT_BRINGUP_LAUNCH is not defined..."
    exit
  fi
  echo "Launching robot $ROBOT_BRINGUP_PACKAGE/$ROBOT_BRINGUP_LAUNCH..."
  screen -S bringup -d -m rosrun rosmon rosmon --name=rosmon_bringup $ROBOT_BRINGUP_PACKAGE $ROBOT_BRINGUP_LAUNCH;

fi

sleep 5;

if $ROBOT_RUN_SENSORS
then
  echo_success "Launching sensors..."
  screen -S sensors -d -m rosrun rosmon rosmon --name=rosmon_sensors robot_bringup sensors_complete.launch;
  if [[ ${ROBOT_RUN_NTRIP} && ${ROBOT_GPS_MODEL} != "none" ]]
  then
    if [[ -z "${ROBOT_GPS_NTRIP_USER}" ]] 
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_NTRIP_USER is not defined"
    elif [[ -z "${ROBOT_GPS_NTRIP_PASSWORD}" ]]
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_NTRIP_PASSWORD is not defined"
    elif [[ -z "${ROBOT_GPS_NTRIP_SERVER}" ]]
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_NTRIP_SERVER is not defined"
    elif [[ -z "${ROBOT_GPS_NTRIP_PORT}" ]]
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_NTRIP_PORT is not defined"
    elif [[ -z "${ROBOT_GPS_NTRIP_AREA_CORRECTIONS}" ]]
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_NTRIP_AREA_CORRECTIONS is not defined"
    elif [[ -z "${ROBOT_GPS_PORT}" ]]
    then
      echo_warning "-> WARNING. The var ROBOT_GPS_PORT is not defined"
    else
      if [[ -f /opt/RTKLIB/app/str2str/gcc/str2str ]]
      then
        echo_success "Launching NTRIP..."
        NTRIP_COMMAND="/opt/RTKLIB/app/str2str/gcc/str2str -in ntrip://${ROBOT_GPS_NTRIP_USER}:${ROBOT_GPS_NTRIP_PASSWORD}@${ROBOT_GPS_NTRIP_SERVER}:${ROBOT_GPS_NTRIP_PORT}/${ROBOT_GPS_NTRIP_AREA_CORRECTIONS} -out serial://${ROBOT_GPS_PORT}:9600 8N:1"
        screen -S NTRIP -d -m $NTRIP_COMMAND
      else
        echo_error "ERROR. RTKLIB is not installed in /opt/RTKLIB"
      fi
    fi
  fi
fi

if $ROBOT_RUN_LOCALIZATION
then
  echo_success "Launching localization packages..."
  screen -S localization -d -m rosrun rosmon rosmon --name=rosmon_localization --no-start robot_bringup localization_complete.launch;
  sleep 2;
fi


if $ROBOT_RUN_PERCEPTION
then
  echo_success "Launching perception packages..."
  screen -S perception -d -m rosrun rosmon rosmon --name=rosmon_perception robot_bringup perception_complete.launch;
  sleep 2;
fi

if $ROBOT_RUN_NAVIGATION
then
  echo_success "Launching navigation packages..."
  screen -S navigation -d -m rosrun rosmon rosmon --name=rosmon_navigation robot_bringup navigation_complete.launch;
  sleep 2;
fi

if $ROBOT_RUN_ROBOT_LOCAL_CONTROL
then
  echo_success "Launching robot_local_control packages..."
  screen -S rlc -d -m rosrun rosmon rosmon --name=rosmon_rlc robot_bringup robot_local_control.launch;
  sleep 2;
fi

if $ROBOT_RUN_HMI
then
  echo_success "Launching hmi packages..."
  screen -S hmi -d -m rosrun rosmon rosmon --name=rosmon_hmi robot_bringup hmi_complete.launch;
  sleep 2;
fi

if $ROBOT_RUN_COMMAND_MANAGER
then
  echo_success "Launching command manager..."
  screen -S command_manager -d -m rosrun rosmon rosmon --name=rosmon_command_manager robot_bringup command_manager_complete.launch;
fi

if $ROBOT_RUN_ARM
then
  echo_success "Launching arm..."
  sleep 2;
  screen -S arm_bringup -d -m rosrun rosmon rosmon --name=rosmon_arm_bringup robot_bringup manipulation_complete.launch;
fi

if $ROBOT_RUN_ROSBAG_MANAGER
then
  echo_success "Launching rosbag manager..."
  screen -S rosbag_manager -d -m rosrun rosmon rosmon --name=rosmon_rosbag_manager $ROBOT_BRINGUP_PACKAGE rosbag_manager.launch;
fi

if $ROBOT_RUN_INTERFACES
then
  echo_success "Launching interfaces..."
  screen -S interfaces -d -m rosrun rosmon rosmon --name=rosmon_interfaces $ROBOT_BRINGUP_PACKAGE interfaces_complete.launch;
fi

if $ROBOT_RUN_SAFETY
then
  echo_success "Launching safety..."
  screen -S safety -d -m rosrun rosmon rosmon --name=rosmon_safety $ROBOT_BRINGUP_PACKAGE safety_complete.launch;
fi
if $ROBOT_RUN_EXTERNAL_MACHINE
then
  if [[ ${ROBOT_EXTERNAL_MACHINE_1_USER} != "none" ]]
  then
    echo_success "Launching external machine 1..."
    if ssh $ROBOT_EXTERNAL_MACHINE_1_USER@$ROBOT_EXTERNAL_MACHINE_1_IP $ROBOT_EXTERNAL_MACHINE_1_COMMAND
    then
      echo_success "Command started in external machine 1";
    else
      echo_error "Error communicating with external machine 1";
    fi
  fi
  if [[ ${ROBOT_EXTERNAL_MACHINE_2_USER} != "none" ]]
  then
    echo "Launching external machine 2..."
    if ssh $ROBOT_EXTERNAL_MACHINE_2_USER@$ROBOT_EXTERNAL_MACHINE_2_IP $ROBOT_EXTERNAL_MACHINE_2_COMMAND
    then
      echo_success "Command started in external machine 2";
    else
      echo_error "Error communicating with external machine 2";
    fi
  fi
fi
