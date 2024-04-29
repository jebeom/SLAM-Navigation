# Update container
sudo apt-get update

# Download dependencies
rosdep update
rosdep install --from-paths ${CATKIN_WS}/src --ignore-src --rosdistro=${ROS_DISTRO} -y -r

# Copy .vscode folder to workspace
ABS_SCRIPT_PATH="$(realpath "${BASH_SOURCE:-$0}")"
ABS_DIRECTORY="$(dirname "${ABS_SCRIPT_PATH}")"
ln -s ${ABS_DIRECTORY}/../.vscode $PWD
