# robot_simple_command_manager

This package contains a node (**robot_simple_command_manager**) that uses different interfaces (**handlers**) that allow the use of services, actions and Robotnik procedures from different ROS nodes accepting simple commands with the following structure: **TAG \<arg1> ... \<argN>** through a standard ROS topic, service or action.

## Index
  - [Dependencies](#dependencies)
  - [Description](#description)
    - [Command Manager](#command-manager)
      - [Parameters](#parameters)
      - [Subscriptions](#subscriptions)
      - [Publications](#publications)
      - [Services](#services)
      - [Action Server](#action-server)
    - [Handlers](#handlers)
      - [Configuration](#configuration)
  - [User Guide](#user-guide)
    - [Sending commands](#sending-commands)
    - [Loading a handler](#loading-a-handler)
  - [Developer Guide](#developer-guide)

---

## Dependencies

The package depends on some Robotnik packages:

- rcomponent [🔗](https://github.com/RobotnikAutomation/rcomponent/)

```bash
git clone https://github.com/RobotnikAutomation/rcomponent/
```

- robot_simple_command_manager_msgs[🔗](https://github.com/RobotnikAutomation/robot_simple_command_manager_msgs)

```bash
git clone https://github.com/RobotnikAutomation/robot_simple_command_manager_msgs
```

- robot_local_control_msgs[🔗](https://github.com/RobotnikAutomation/robot_local_control_msgs)

```bash
git clone https://github.com/RobotnikAutomation/robot_local_control_msgs
```

This package may depend on other Robotnik or ROS standard packages in function of the handlers that are developed. The standard ROS packages can be installed using the rosdep install command:

```bash
# located in the workspace folder
rosdep install --from-path src --ignore-src -y -r
```

---

## Description

The command manager node is separated in two key elements: the command manager itself and the handlers.

### Command Manager

The robot_simple_command_manager loads handlers that are capable of parsing simple string-encoded commands to ROS messages.

The commands are structured as follows:

```bash
<TAG> [arguments]
```

Where the **\<TAG>** is the name that we have configured for a specific implementation of a handler and the **[arguments]** is a list of the arguments (required or not) that the handler needs to perform the action or service.

#### Parameters

- **command_name** (string): Name that the node will provide to the topic and service to send commands.

- **base_command_list** (string[]): List with the name of the desired handlers*. The purpose of this list is to include all commands considered common or standar to all robots.

- **command_list** (string[]): List with the name of the desired handlers*. The purpose of this list is to include only the commands considered specific to the application or the robot model.

\* The configuration of the desired handlers needs to be provided to be load.

Here is an example of a node configuration that has load two different types of handlers to be capable to perform the commands GOTO and ELEVATOR:

```yaml
# All robots may be capable to use a move_base action
base_command_list:
  - GOTO

# Only robots with an elevator are capable to interact with this component
command_list:
  - ELEVATOR  

GOTO:
  type: move_base_action_interface/MoveBaseActionInterface
  namespace: move_base
  frame_id: rb1_base_map

ELEVATOR:
  type: elevator_service_interface/ElevatorServiceInterface
  namespace: robotnik_base_control/set_elevator
```

This node also has all the parameters available from its parent node, [RComponent](https://github.com/RobotnikAutomation/rcomponent/blob/master/src/rcomponent/rcomponent.py).

#### Subscriptions

- **command** (robot_simple_command_manager_msgs/CommandString): Topic to receive commands.

- **cancel** (std_msgs/Empty): Topic to cancel commands.

The robot_simple_command_manager_node may be subscribed also to the feedback, result and status topics that the actions handlers use.

#### Publications

- **state** (robotnik_msgs/State): Publishes the current state of the node

- **feedback** (robot_simple_command_manager_msgs/CommandStringFeedback): Publishes the feedback related to the current command sent.

The robot_simple_command_manager_node may also publish to the cancel and goal topics that the actions handlers use.

#### Services

- **command** (robot_simple_command_manager_msgs/SetCommandString): Service to receive commands.

- **cancel** (std_srvs/Trigger): Service to cancel commands.

- **get_handlers** (robot_simple_command_manager_msgs/GetHandlers): Returns a list of the loaded handlers

- **get_handler_info** (robot_simple_command_manager_msgs/GetHandlerInfo): Returns the description and type of the arguments of the command. It also returns the description and type of the output of the command.

- **get_handler_info_list** (robot_simple_command_manager_msgs/GetHandlerInfoList): Returns the description and type of the arguments of all (or a selective list of) the commands. It also returns the description and type of the output of the command.

#### Action Server

- **action** (robot_simple_command_manager_msgs/CommandStringActionAction): Action server to receive commands.

---

### Handlers

The handlers are responsible for parsing the string-encoded command to ROS messages.

#### Configuration

The handlers need to be configured through parameters as shown below:

```yaml
TAG:
  type: python_module/PythonClass
  namespace: target_namespace
```

There are some general parameters that need to be set for any handler:

- **TAG**: This is the keyword that will be used to identify the type of command sent to the robot_simple_command_manager.

- **type**: Identifies which module (It matches the name of the python script where the handler is) and class (It matches the name of the python class that is in the module) needs the robot_simple_command_manager to load when the handler is added to the command_handlers parameter. You can also load handlers from external packages (More info in the developer section).

- **namespace**: Correspond to the action or service namespace that the handler client needs to perform the command.

- **communication_timeout**: Action handlers accept this parameter to set the maximum time without receiving feedback from action server. If it is set to a value lower or equal to 0.0, there will not be communication monitoring.
Specific parameters can be also defined as shown below:

```yaml
GOTO:
  type: move_base_action_interface/MoveBaseActionInterface
  namespace: move_base
  communication_timeout: 10
  frame_id: rb1_base_map
```

The GOTO handler has the common parameters as well as an specific parameter **frame_id**.

---

## User Guide

First it is necessary to launch the node. It will automatically load the base_commands.yaml and commands.yaml files from the config folder:

```bash
roslaunch robot_simple_command_manager robot_simple_command_manager.launch
```

### Sending commands

You can send commands through topic:

```bash
rostopic pub /command_manager/command robot_simple_command_manager_msgs/CommandString "command: 'GOTO 0.0 0.0 0.0'"
```

service:

```bash
rosservice call /command_manager/command "command: 'GOTO 0.0 0.0 0.0'"
```

or action:

```bash
rostopic pub /command_manager/action/goal robot_simple_command_manager_msgs/RobotSimpleCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    command: 'GOTO 0.0 0.0 0.0'"

```

For any case you will need to provide an available TAG (in this case GOTO) that has been defined in the handlers.yaml file with its required arguments.

Some aspects to take into account are:

- The **commands are not queued** in any case.

- **Sending a command does not overwrite** a command that is currently running.

- To overwrite a command, **you must first explicitly cancel it**.

You can cancel commands through topic:

```bash
rostopic pub /rb1_base/command_manager/cancel std_msgs/Empty "{}"
```

service:

```bash
rosservice call /rb1_base/command_manager/cancel "{}"
```

or action (this only cancels action goals):

```bash
rostopic pub /rb1_base/command_manager/action/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"
```

Regardless of the interface through which the user has sent the command (topic, action or service), it can be cancelled as the user prefers (topic, action or service).

### Loading a handler

To load a handler you will first need to add it to the list of [command_list](#parameters) in the commands.yaml file in the config folder. And then you will need to add its [configuration](#configuration).

Things to consider when setting up a handler:

- All the available handlers are located in the [handlers](src/robot_simple_command_manager/handlers/) folder.
- Handlers can be located in external packages instead of main command_manager package.
- Parameters:
  - The **type** is defined by the python script filename where the handler is located (wihout the .py extension), followed by a __/__ and finished by the name of the Python class inside that script. For example, if we wanted to load a handler called ElevatorServiceInterface that is located in the elevator_service_interface.py the type would be:

    ```yaml
    type: elevator_service_interface/ElevatorServiceInterface
    ```

  - If the handler is located in an external package, you need to prepend the custom package name and add a _/_ to the type of the handler. For example, if we wanted to load a handler called ElevatorServiceInterface that is located in the elevator_service_interface.py inside the my_custom_package the type would be:
  
    ```yaml
    type: my_custom_package/elevator_service_interface/ElevatorServiceInterface
    ```
 
  - The **namespace** is the name that the handler client will use to connect to the service, action or topic to perform the command.

    - For a service or topic, the namespace equals to the name of the service.
  
    - For an action, the namespace equals to the name of the action without the suffixes (cancel, feedback, goal and result). The handler automatically connect to that topics.

    - For a procedure, the namespace equals to the name of the procedure without the robot_local_control/NavigationComponent prefix and the suffixes (query_state, add or cancel).

For example, to connect to the elevator service the namespace would be:

```yaml
namespace: set_elevator
```

To connect to robot_local_control/GoToProcedure:

```yaml
namespace: GoToProcedure
```

To connect to move_base server:

```yaml
namespace: move_base
```

---

## Developer Guide

The most common development use case for the Robot Simple Command Manager is to create a new handler to interact with a new type of ROS message (service, topic or action).
In this context, it is not necessary to modify the main package to add project-specific functionality. The component has been prepared to be able to load new handlers defined in external packages. For the development of these packages it is recommended to use the package [scripts](scripts), which will make this task easier.

Use the bash script to create a new ROS packages with CM structure to add new handlers
```bash
./create_handlers_package.sh
```

Use the python script to create a new handler. This script will invoke a wizard that will ask for the different parameters required to configure your handler
```bash
python create_handler.py
```
After creating the new handler you may move it to the custom package.

Remeber that if you want to use handlers that are not located in the main Robot Simple Command Manager, you may need to specify the package name in the type parameter when defining the different handlers.
