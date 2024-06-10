## SLAM & Navigation

I used the [cartographer](https://github.com/cartographer-project/cartographer_ros) and [gmapping](https://github.com/ros-perception/slam_gmapping) to perform SLAM. The mobile robot is [Robotnik's](https://robotnik.eu/) SUMMIT-XL STEEL, and the manipulator is [Franka's](https://franka.de/) FR3.

This repository is for **Real world** purposes.

If you want to test in a Simulation environment, See the **Simualtion branch**


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
mv KIST-SLAM-Navigation src
catkin build && source devel/setup.bash
```
### Lidar Sensor Merge
```
roslaunch summit_xl_bringup merge_laser.launch
```
### Run amcl & map_server node 
```
roslaunch summit_xl_localization moma_localization.launch
```
### Run move_base node
```
roslaunch summit_xl_navigation moma_move_base.launch
```
After executing the commands above, launch Rviz and change the fixed frame to **robot_map**, add the **map topic**, and set a destination using the **2D Nav Goal** to enable autonomous navigation to that target. 

Additionally, by visualizing topics such as the local costmap, global costmap, and the paths from the local and global planners in Rviz, you can gain a more accurate understanding of how the robot is moving.
