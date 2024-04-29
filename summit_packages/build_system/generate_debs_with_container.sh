#!/bin/bash

# This script should be run from catkin workspace root

function print_info {
    echo -e "\e[1;34m[INFO]: $1\e[0m"
}

function print_error {
    echo -e "\e[1;31m[ERROR]: $1\e[0m"
}

function ask {
    question=$1
    default_answer=${2:-1}
    while true; do
        read -p "$(echo -e "\e[1;33m[ASK]:\e[0m ${question} [Y/n]: ")" yn
        case $yn in
            [Yy]* ) return 0;;
            "" ) return "${default_answer}";;
            [Nn]* ) return 1;;
            * ) echo -e "\e[1;31m[ERROR]:\e[0m Please answer yes or no.";;
        esac
    done
}

function print_usage() {
    echo "Usage: generate_debs_with_container.sh [OPTIONS]"
    echo "Options:"
    echo "  -h, --help: show this help"
    echo "  -c, --container: id of the container to use (default: robotnik_deb_generator_container)"
    echo "  -w, --rewrite-whitelist: generate whitelist with private folder"
    echo "  -p, --pcan-version: version of the pcan driver to install (default: get version from pcaninfo)"
}

# Args parsing
id_container="robotnik_deb_generator_container"
rewrite_whitelist=0
pcan_version=""
packages_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"
distro="$(echo $ROS_DISTRO)"
image_name="robotnik/ros:${distro}-builder"

function parse_args() {
    while [ "$1" != "" ]; do
        case $1 in
            -c | --container )      shift
                                    id_container=$1
                                    ;;
            -w | --rewrite-whitelist )  rewrite_whitelist=1
                                    ;;
            -p | --pcan-version )   shift
                                    pcan_version=$1
                                    ;;
            -h | --help )           print_usage
                                    exit 0
                                    ;;
            * )                     print_usage
                                    exit 1
        esac
        shift
    done

    if [ -z "$pcan_version" ]; then
        pcan_version=$(pcaninfo | grep -oP '(?<=driver version: )\d+\.\d+\.\d+')
        if [ -z "$pcan_version" ]; then
            print_error "PCAN driver not found"
            exit 1
        fi
    fi
}

# Parse args
parse_args "$@"

# Check if container exists and create it if not
if [ -z "$(docker ps -a | grep "${id_container}")" ]; then
    print_info "Creating container ${id_container}"
    print info "Mounting packages dir: ${packages_dir}"
    docker run -d --name "${id_container}" -u1000 -v "${packages_dir}/:/home/robot/robot_ws/src/robot_packages" "${image_name}" sleep infinity

    # Check if pcan driver is installed
    if [ -z "$(docker exec -it "${id_container}" pcaninfo | grep "${pcan_version}")" ]; then
        print_info "Installing PCAN driver version ${pcan_version}"
        if ! docker exec -it "${id_container}" install_pcan.sh -p "${pcan_version}"; then
            print_error "Error installing PCAN driver"
            exit 1
        fi
    fi

    # Install dependencies (follow internal guide)
    print_info "Installing dependencies step 5.1."
    if ! docker exec -it "${id_container}" sudo apt install -y \
        "ros-${distro}-transmission-interface" \
        "ros-${distro}-effort-controllers" \
        "ros-${distro}-joint-state-controller" \
        "ros-${distro}-navigation" \
        "ros-${distro}-ros-control" \
        "ros-${distro}-ros-controllers" \
        "ros-${distro}-velocity-controllers" \
        "ros-${distro}-control-toolbox" \
        "ros-${distro}-cmake-modules" \
        "ros-${distro}-serial" \
        "ros-${distro}-rosbridge-server" \
        "ros-${distro}-robot-localization" \
        "ros-${distro}-twist-mux" \
        "ros-${distro}-imu-tools" \
        "ros-${distro}-mavros-"* \
        "ros-${distro}-teb-local-planner" \
        "ros-${distro}-ackermann"* \
        "ros-${distro}-rosbridge-suite" \
        "ros-${distro}-urg-node" \
        "ros-${distro}-move-base" \
        "ros-${distro}-amcl" \
        "ros-${distro}-libuvc-camera" \
        "ros-${distro}-realsense2-camera" \
        "ros-${distro}-rqt-joint-trajectory-controller" \
        "ros-${distro}-psen-scan"* \
        "ros-${distro}-joy" \
        "ros-${distro}-ddynamic-reconfigure" ; then
        print_error "Error installing step 1 dependencies"
        exit 1
    fi
    print_info "Installing dependencies step 5.2."
    if ! docker exec -it "${id_container}" sudo apt install -y \
        libpopt-dev \
        libpcap-dev \
        ffmpeg \
        libmodbus* \
        libmysql++3v5 \
        libmysql++-dev ; then
        print_error "Error installing step 2 dependencies"
        exit 1
    fi
    print_info "Installing EthernetIP"
    if ! docker exec -it "${id_container}" bash -c "cd /tmp && git clone https://github.com/nimbuscontrols/EIPScanner && cd EIPScanner && git checkout 1.3.0 && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && rm -rf /tmp/EIPScanner"; then
        print_error "Error installing EthernetIP"
        exit 1
    fi
    print_info "Installing libuvc (for astra camera)"
    if ! docker exec -it "${id_container}" bash -c "sudo apt-get install -y ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev && cd /tmp && git clone https://github.com/libuvc/libuvc.git && cd libuvc && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && rm -rf /tmp/libuvc && cd && sudo ldconfig"; then
        print_error "Error installing libuvc (for astra camera)"
        exit 1
    fi
else
    print_info "Container ${id_container} already exists"
fi

# Check if container is running
if [ -z "$(docker ps | grep "${id_container}")" ]; then
    print_info "Starting container ${id_container}"
    docker start "${id_container}"
fi

# Generate local dependencies
print_info "Generating local dependencies"
if ! docker exec -it "${id_container}" local_deps.sh; then
    print_error "Error installing local dependencies"
    exit 1
fi

# Install dependencies from rosdep
print_info "Updating rosdep"
if ! docker exec -it "${id_container}" rosdep update; then
    print_error "Error executing rosdep update"
    exit 1
fi

print_info "Installing dependencies from rosdep"
if ! docker exec -it "${id_container}" rosdep install --from-paths src --ignore-src -r -y; then
    print_error "Error installing dependencies from rosdep"
    exit 1
fi
#libpcl-dev da error de keyboard
# Compile workspace
print_info "Compiling workspace"
if ! docker exec -it "${id_container}" compile_workspace.sh; then
    print_error "Error compiling workspace"
    exit 1
fi

# Remove previous debs
print_info "Removing previous debs"
if ! docker exec -it "${id_container}" rm -rf /home/robot/robot_ws/debs/*; then
    print_error "Error removing previous debs"
    exit 1
fi

# Generate whitelist
if [ "${rewrite_whitelist}" == "1" ]; then
    print_info "Rewriting whitelist"
    if ! docker exec -it "${id_container}" generate_whitelist.sh; then
        print_error "Error rewriting whitelist"
        exit 1
    fi
else
    print_info "Copying whitelist from package deb folder"
    if ! docker cp  "${packages_dir}/deb/whitelist.txt" "${id_container}":/home/robot/robot_ws/debs/whitelist.txt; then
        print_error "Error copying whitelist from package deb folder"
        exit 1
    fi
fi

# Generate debs
print_info "Generating debs"
if ! docker exec -it "${id_container}" generate_debs.sh; then
    print_error "Error generating debs"
    exit 1
fi

# Remove previous package deb folder
if [ -d "${packages_dir}/deb" ]; then
    print_info "Removing previous deb folder"
    if ! rm -rf "${packages_dir}/deb"; then
        print_error "Error removing previous deb folder"
        exit 1
    fi
fi

# Copy debs to host
print_info "Copying debs to host"
if ! docker cp "${id_container}":/home/robot/robot_ws/debs/. "${packages_dir}/deb"; then
    print_error "Error copying debs to host"
    exit 1
fi

# Remove ddebs from debs folder
print_info "Removing ddebs from deb folder"
if ! rm -rf "${packages_dir}/deb"/*.ddeb; then
    print_error "Error removing ddebs from debs folder"
    exit 1
fi

# Create install_debs.sh and remove_debs.sh
cat <<EOF > "${packages_dir}/deb/install_debs.sh"
#!/bin/bash
# Check if script is being run as root
if [ "\$EUID" -ne 0 ]; then
    echo -e "\e[1;31m[ERROR]:\e[0m Please run as root"
    exit 1
fi
script_dir="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo -e "\e[1;34m[INFO]:\e[0m Running: apt update"
apt update
echo -e "\e[1;34m[INFO]:\e[0m Running: apt install \${script_dir}/*.deb -y"
if ! apt install "\${script_dir}"/*.deb -y; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error installing debs, please check package.xml to see if all dependencies are satisfied"
    exit 1
fi
EOF
chmod +x "${packages_dir}/deb/install_debs.sh"

cat <<EOF > "${packages_dir}/deb/remove_debs.sh"
#!/bin/bash
# Check if script is being run as root
if [ "\$EUID" -ne 0 ]; then
    echo -e "\e[1;31m[ERROR]:\e[0m Please run as root"
    exit 1
fi
script_dir="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# List of packages of debs to remove
packages_to_remove=""
files=("\${script_dir}"/*.deb)
for file in "\${files[@]}"; do
    packages_to_remove="\${packages_to_remove} \$(dpkg-deb -f "\${file}" Package)"
done
echo -e "\e[1;34m[INFO]:\e[0m Running: apt remove \${packages_to_remove} -y"
if ! apt remove \${packages_to_remove} -y; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing debs"
    exit 1
fi
EOF
chmod +x "${packages_dir}/deb/remove_debs.sh"
