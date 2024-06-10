# robotnik_urcap_ros

Packages to interact with the RB-Kairos URCap.

## Dependnecies

To use this package you will need to have the following Robotnik packages installed:

- [xmlrpc_server](https://github.com/RobotnikAutomation/xmlrpc_server)

- [rcomponent](https://github.com/RobotnikAutomation/rcomponent)

- [robot_simple_command_manager](https://github.com/RobotnikAutomation/robot_simple_command_manager)

- [robot_simple_command_manager_msgs](https://github.com/RobotnikAutomation/robot_simple_command_manager_msgs)

- [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs)


## robotnik_urcap_bringup

Contains the configuration and launch files that allow to receive the URCap commands through the xmlrpc server

## robotnik_urcap_bridge

XMLRPC server to communicate with Polyscope

## robotnik_urcap_bridge_msgs

Package with custom messages used to communicate with Polyscope 

### Launch example

```bash
roslaunch robotnik_urcap_bringup urcap_complete.launch
```
