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

robot_params="/home/robot/robot_params"


#check if $robot_params file exists.
if [ -d "$robot_params" ]; then
  echo_success "->Loading robot_params from $robot_params"
  if [ "$?" != "0" ]; then
    echo_error "Error reading folder!! $robot_params"
    return -1
  fi
else
# Loads params using the path of the current script
  echo_warning "$robot_params doesn't exist. Loading config from robot_bringup/env"
  robot_params="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../env"
  if [ -d "$robot_params" ]; then
    echo_success "Loading robot_params from $robot_params"
    if [ "$?" != "0" ]; then
        echo_error "Error reading file!! $robot_params"
        return -1
    fi
  fi
fi


function source_file() {
  if [ -f "$1" ]; then
    echo_success "-> Read $1"
    if [ "$?" != "0" ]; then
      echo_error "-> Error reading file!! $1"
      return -1
    fi
    source $1
  else
    echo_error "-> File $1 doesn't exist"
    return -1
  fi
}

echo_warning "All the env variables configuration are located at $robot_params"

source_file $robot_params/bringup.env
source_file $robot_params/robot_params.env
source_file $robot_params/sensors_params.env
source_file $robot_params/battery_params.env
source_file $robot_params/localization_params.env
source_file $robot_params/manipulation_params.env
source_file $robot_params/navigation_params.env
source_file $robot_params/perception_params.env
source_file $robot_params/rlc_params.env
source_file $robot_params/simulation_params.env
source_file $robot_params/hmi_params.env
source_file $robot_params/interfaces_params.env
source_file $robot_params/safety_params.env
source_file $robot_params/external_machine_params.env

export ROS_MASTER_URI=http://$HOSTNAME:11311
export ROS_HOSTNAME=$HOSTNAME
source_file /opt/ros/$ROS_DISTRO/setup.bash

if [ -d "$ROBOT_WORKSPACE" ]; then
  echo_success "-> Robot workspace is $ROBOT_WORKSPACE"
  if [ "$?" != "0" ]; then
    echo_error "--> Error to get workspace $ROBOT_WORKSPACE . Check the workspace path of the environment variable: ROBOT_WORKSPACE"
    return -1
  fi
else
  echo_error "--> Error to get workspace $ROBOT_WORKSPACE . Check the workspace path of the environment variable: ROBOT_WORKSPACE"
  return -1
fi
source_file $ROBOT_WORKSPACE/devel/setup.bash

echo ""
echo -e "ROS_MASTER_URI\t= $ROS_MASTER_URI"
echo -e "ROS_HOSTNAME\t= $ROS_HOSTNAME"
echo -e "WORKSPACE \t= $ROBOT_WORKSPACE/devel/setup.bash"
echo ""
