#!/bin/bash

function print_info {
    echo -e "\e[1;34m[INFO]:\e[0m $1"
}

function print_error {
    echo -e "\e[1;31m[ERROR]:\e[0m $1"
}

function ask {
    question=$1
    default_answer=${2:-1}
    while true; do
        if [ "${default_answer}" == "1" ]; then
            read -p "$(echo -e "\e[1;33m[ASK]:\e[0m ${question} [y/N]: ")" yn
        else
            read -p "$(echo -e "\e[1;33m[ASK]:\e[0m ${question} [Y/n]: ")" yn
        fi
        case $yn in
            [Yy]* ) return 0;;
            "" ) return "${default_answer}";;
            [Nn]* ) return 1;;
            * ) echo -e "\e[1;31m[ERROR]:\e[0m Please answer yes or no.";;
        esac
    done
}

id_container=${1:-"robotnik_deb_generator_container"}

# Check if container exists
if [ -z "$(docker ps -a | grep "${id_container}")" ]; then
    print_error "Container ${id_container} does not exist"
    exit 1
fi

# Ask if user wants to remove the container
if ask "Do you want to remove the container ${id_container}?" 1; then
    print_info "Removing container ${id_container}"
    if ! docker rm -f "${id_container}"; then
        print_error "Error removing container ${id_container}"
        exit 1
    fi
fi
