## SLAM & Navigation

I used the [cartographer](https://github.com/cartographer-project/cartographer_ros) and [gmapping](https://github.com/ros-perception/slam_gmapping) to perform SLAM. The mobile robot is [Robotnik's](https://robotnik.eu/) SUMMIT-XL STEEL, and the manipulator is [Franka's](https://franka.de/) FR3. 

This repository is for **simulation** purposes.

### Software selection -- OS:
Currently tested and working configurations:

- Ubuntu 20.04 + ROS 1 noetic


---
## Quick Run 
### Make Work Space
```
mkdir <your-catkin-work-space>
cd <your-catkin-work-space>
git clone <KIST-SLAM-Navigation_repo_url>
cd <KIST-SLAM-Navigation_repo_url>
mv <KIST-SLAM-Navigation_repo_url> src
cd src
```
### Checkout Simulation branch and build 

```
git checkout Simulation
catkin build && source devel/setup.bash
```
### Simulation(Rviz & Gazebo)
```
roslaunch summit_xl_gazebo summit_xl_gazebo.launch
```

