#!/bin/bash

# script intended to release a new version into the release repository
# arguments: package_folder_name
#            - name of the folder is going to be created to save the package
OUTPUT_RED='\033[0;31m'
OUTPUT_NC='\033[0m'
OUTPUT_GREEN='\033[0;32m'
OUTPUT_ORANGE='\033[0;33m'
OUTPUT_YELLOW='\033[1;33m'

echo -e "${OUTPUT_ORANGE}"
echo -e "#################################################################################"
echo -e "This scripts has to be run on the root of the respository"
echo -e "It will create a new temporal folder"
echo -e "[Optional] It will build the .debs of all the packages"
echo -e "   You can define a list of blacklisted package in the env var DEB_BUILDER_BLACKLIST"
echo -e "It will remove the private packages"
echo -e "It will remove the build_system package"
echo -e "It will remove the git info of the package, but it will keep submodules git info"
echo -e "#################################################################################"
echo -e "${OUTPUT_NC}"

export DEB_BUILD_OPTIONS="parallel=`nproc`"
echo -e "${OUTPUT_YELLOW}I'm setting the env var DEB_BUILD_OPTIONS to $DEB_BUILD_OPTIONS ${OUTPUT_NC}"


if [ "$#" -ne 1 ]; then
    # rbkairos_constructa
    ROOT_PACKAGE_NAME=$(basename `git rev-parse --show-toplevel`)
    # devel, master, tag...
    BRANCH=$(git rev-parse --abbrev-ref HEAD)
    # name of the folder to create the new release workspace
    NEW_PACKAGE_FOLDER=$ROOT_PACKAGE_NAME
    echo -e "Using default name for the package folder ... [$NEW_PACKAGE_FOLDER]"
else
    NEW_PACKAGE_FOLDER=$1
fi

ROOT_PATH=$(pwd)

echo -e "${OUTPUT_YELLOW}Do you want to build the libraries for $ROS_DISTRO (or use the current ones)? (yes/NO):${OUTPUT_NC}"
read response
if [ $response = "yes" ]
then
  export DEB_BUILD_OPTIONS="parallel=`nproc`"
  echo -e "Setting DEB_BUILD_OPTIONS to $DEB_BUILD_OPTIONS..."
  sleep 2
  # Install deps
  echo -e "Building the libraries..."
  if [ -z "${DEB_BUILDER_BLACKLIST}" ]; then
    DEB_BUILDER_BLACKLIST=""
    echo -e "There's no a blacklist of packages to avoid building..."
    sleep 2
  fi

  #./scripts/generate_debs.sh
  sudo ./build_system/rosdep_creator.py --ros-distro $ROS_DISTRO
  rosdep update
  eval "sudo ./build_system/builder.py --ros-distro $ROS_DISTRO --blacklist \"$DEB_BUILDER_BLACKLIST\""
  if [ "$?" != "0" ]; then
      echo -e "${OUTPUT_RED}Error building packages!${OUTPUT_NC}"
      return -1
  fi
fi

echo -e "Creating temporal folder to save the release data $NEW_PACKAGE_FOLDER"
sleep 1

# Removes previous ones..
if [ -d "$NEW_PACKAGE_FOLDER" ]; then
    echo -e "Removing release folder $NEW_PACKAGE_FOLDER"
    sleep 1
    rm -rf $NEW_PACKAGE_FOLDER
fi

if [ "$?" != "0" ]; then
    echo -e "${OUTPUT_RED}Error removing $NEW_PACKAGE_FOLDER ${OUTPUT_NC}"
    return -1
fi

if [ -f "$NEW_PACKAGE_FOLDER.tar.gz" ]; then
    echo -e "Removing release compressed file $NEW_PACKAGE_FOLDER.tar.gz"
    sleep 1
    rm -rf $NEW_PACKAGE_FOLDER.tar.gz
fi

if [ "$?" != "0" ]; then
    echo -e "${OUTPUT_RED}Error removing $NEW_PACKAGE_FOLDER.tar.gz${OUTPUT_NC}"
    return -1
fi

echo -e "Creating folder $NEW_PACKAGE_FOLDER"
mkdir $NEW_PACKAGE_FOLDER
sleep 1

if [ "$?" != "0" ]; then
    echo -e "${OUTPUT_RED}Error creating $NEW_PACKAGE_FOLDER ${OUTPUT_NC}"
    return -1
fi


# copy everything except private folder
rsync -a . ./$NEW_PACKAGE_FOLDER --exclude private --exclude $NEW_PACKAGE_FOLDER

if [ "$?" != "0" ]; then
    echo -e "${OUTPUT_RED}Error copying files to folder $NEW_PACKAGE_FOLDER ${OUTPUT_NC}"
    return -1
fi

echo -e "Move to $NEW_PACKAGE_FOLDER"
cd ./$NEW_PACKAGE_FOLDER

echo -e "Removing all the root git info..."
( find . -type d -name ".git" && find . -name ".gitignore" && find . -name ".gitmodules" ) | xargs rm -rf
echo -e "Removing build_system package..."
rm -rf ./build_system

cd $ROOT_PATH
echo -e "Creating compressed folder..."
tar -czf $NEW_PACKAGE_FOLDER.tar.gz $NEW_PACKAGE_FOLDER

echo -e "${OUTPUT_GREEN}New package $NEW_PACKAGE_FOLDER created with debs!${OUTPUT_NC}"
