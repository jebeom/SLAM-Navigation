# build_system

This package provide a set of tool for build ROS packages and workspaces as well as others related to package versions generation.

## 1. Installation, dependencies and configuration

 1. Install the "python-catkin-tools" package
    ```
    sudo apt-get install python-catkin-tools
    ```

 2. Build and source the workspace with "catkin build"
    ```
    catkin build
    source devel/setup.bash
    ```

## 2. Scripts


### 2.1 rosdep_creator.py

Shows configuration steps for rosdep and outputs the dependencies to be added to the local rosdep file, according to the content of the workspace.

### 2.2 builder.py

Automates debian binary creation: creates debs, installs them and deletes intermediate files. Moreover, traverses them in topological order, so there are no race conditions between not-yet build dependencies.

### 2.3 generate_version.sh (deprecated!)

This script is intended to automate the package version generation, building the .deb files (using the .py files that are on this package), removing the private packages and git files, and compressing the workspace on a .tar.tgz file.

### 2.4 create_package_with_debs.sh

It creates a prerelease package to test it with libraries/debs and without private packages

### 2.5 release_version.sh

It combines the package creation with debs together with the publication into a release repository.

### 2.6 get_raw_changelog.sh

It gets a formatted changelog based on commits of the current repository. It can be used to create changelog manually.

## 3. How to use

### 3.1 Automatic package creation and release

1. Execute the "create_package_with_debs.sh" file with sudo permissions.
   ```
   ./build_system/create_package_with_debs.sh
   ```
   Set the env var **DEB_BUILDER_BLACKLIST** to add a set of blacklisted packages
   ```
   export DEB_BUILDER_BLACKLIST="package1,package2,..,packageN"
   ```

2. Release.
   ```
   # Usage: ./build_system/release_version.sh tag_version url_to_release_repository

   ./build_system/release_version.sh 1.0.0 https://github.com/RobotnikAutomationRelease/rb1_releases
   ```

### 3.2 Manual package creation

1. Clone this repo in the src directory of the workspace
1. Run **rosdep_creator** from your workspace path
   ```
   ./<path_to_file>/rosdep_creator.py --ros-distro <distro_name>
   ```
1. Execute the command **rosdep update**
   ```
   rosdep update
   ```
1. Run **builder.py** on the path you want the deb folder with the installation and .deb files.
   ```
   ./<path_to_file>/builder.py --ros-distro <distro_name> --blacklist "package1,package2,...,packageN"
   ```


**Parameters**: the same as bloom-generate, but only ros-distro is mandatory.

* \-\-ros-distro ROS_DISTRO (example: kinetic, melodic)

* \-\-os-name OS_NAME (example: ubuntu)

* \-\-os-version OS_VERSION (example: xenial)

**Additional parameters**:

* \-\-dry-run. Does not execute commands: just to check that it can be run.

* \-\-debug-build. Enables log of build process

* \-\-[no-]install-existing-packages. Installs already built packages

**Others**:

* Set the environment variable ***DEB_BUILD_OPTIONS*** to work with multiple cpus when building the packages if you want to speed up the process. You can specify how many or just use all the available one with *nproc*:
 ```
 export DEB_BUILD_OPTIONS="parallel=`nproc`"
 ```


#### 3.2.1 Examples

~~~~
[/workspaces/test_ws/src/] [19:31]
mbosch@turgon$ git clone https://github.com/RobotnikAutomation/build_system
~~~~

~~~~
[/workspaces/test_ws/src/] [19:31]
mbosch@turgon$ ./build_system/rosdep_creator.py --ros-distro kinetic
~~~~

~~~~
[~/workspaces/test_ws/src/] [19:31]
mbosch@turgon$ ./build_system/builder.py --ros-distro kinetic
~~~~

### 3.3 Manual CHANGELOG generation

```
./build_system/get_raw_changelog.sh [commit_hash]
```

where commit hash indicates the commit to start from. If empty it returns all the log

## TODO

1. Terminal output (completed jobs, add colors, disable output from internal commands, etc) so can be easily understood.
1. Errors from rosdep/build process. Now, as everything is ok, runs perfectly, but in case of bad rosdep/build seems is not going to work.
1. sudo permission.
1. Parallelize building of packages (those whose dependencies are already installed)
1. Scan bringup to extract which packages are to be built (those that are potentially callable from bringup folder)
1. ~~Add timestamp checking, to know what has to be rebuilt.~~
