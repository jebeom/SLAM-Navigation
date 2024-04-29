#!/bin/bash

# This script should be run from catkin workspace root

# Configure

# Get packages dir.
packages_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"
# Get current packages name
packages_name=$(basename "${packages_dir}")
# Get current tag from git, if no tag is found, use the current commit hash from git packages_dir
current_tag=$(cd "${packages_dir}" && git describe --tags --exact-match 2> /dev/null || git rev-parse --short HEAD 2> /dev/null)
# Arch of the packages
packages_arch=$(dpkg --print-architecture)
# Output file name of release folder
tmpdir="${TMPDIR:-/tmp}"
output_folder="${tmpdir}/${packages_name}"
# packages_name changes _ to - to avoid problems with dpkg
output_file="${packages_name//_/-}_${current_tag}_${packages_arch}"


# If release folder exists, remove it
if [ -d "${output_folder}" ]; then
    echo -e "\e[1;34m[INFO]:\e[0m Removing ${output_folder} folder"
    if ! rm -rf "${output_folder}" ; then
        echo -e "\e[1;31m[ERROR]:\e[0m Error removing ${output_folder} folder"
        exit 1
    fi
fi

echo -e "\e[1;34m[INFO]:\e[0m Copying source files"
if ! cp -r "${packages_dir}" "${output_folder}/" ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error copying source files"
    exit 1
fi

# Remove private files
private_src=(
#   "build_system"
    "private"
)
for src in "${private_src[@]}"; do
    echo -e "\e[1;34m[INFO]:\e[0m Removing ${src} folder"
    if ! rm -rf "${output_folder}/${src}" ; then
        echo -e "\e[1;31m[ERROR]:\e[0m Error removing ${src} folder"
        exit 1
    fi
done

echo -e "\e[1;34m[INFO]:\e[0m Removing .git folders"
if ! find "${output_folder}/" -name ".git" -exec rm -rf {} + ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing .git folders"
    exit 1
fi
echo -e "\e[1;34m[INFO]:\e[0m Removing .gitignore files"
if ! find "${output_folder}/" -name ".gitignore" -type f -exec rm -rf {} + ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing .gitignore files"
    exit 1
fi
echo -e "\e[1;34m[INFO]:\e[0m Removing .gitmodules files"
if ! find "${output_folder}/" -name ".gitmodules" -type f -exec rm -rf {} + ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing .gitmodules files"
    exit 1
fi

echo -e "\e[1;34m[INFO]:\e[0m Tarballing ${tmpdir}/${packages_name} into ${output_file}.tar.gz folder"
if ! tar -C "${tmpdir}" -czf "${output_file}.tar.gz" "${packages_name}" ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error tarballing ${output_folder} folder"
    exit 1
fi

echo -e "\e[1;34m[INFO]:\e[0m Removing ${output_folder} folder"
if ! rm -rf "${output_folder}" ; then
    echo -e "\e[1;31m[ERROR]:\e[0m Error removing ${output_folder} folder"
    exit 1
fi
