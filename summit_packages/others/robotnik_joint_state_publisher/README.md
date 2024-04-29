# robotnik_joint_state_publisher

## 1 JointStatePublisher

Node to republish in a single topic a set of sensor_msgs/JointState topics. The publication frequency is set by the node and it republishes the last JointState values received. This node ensures that all joint states are published at the same frequency. In case of a topic timeout (time elapsed without receiving a message), the set of joints that are published by this topic are removed from the output message.

### 1.1 Parameters

* ~**source_list** (List, default: )
  List of topics to be republished. Each item of the list contains the name of the topic and the timeout in seconds.

Example:

```yaml
source_list:
  - name: /joint_state
    timeout: 1.0
  - name: /joint_state_2
    timeout: 1.0
```

This node inherits all the parameters of the [RComponent](https://www.github.com/RobotnikAutomation/rcomponent) base node.

### 1.2 Subscribed Topics

This node subscribes to each topic defined in the source_list parameter.

### 1.3 Published Topics

* ~/**joint_states** (sensor_msgs/JointState)
  Output message that contain the concatenation of the joint states received from the source_list parameter.

### 1.4 Services
This node has not service servers.

### 1.5 Services Called
This node does not call to services.

### 1.6 Action server
This node has not action servers.

### 1.7 Action clients
This node does not call to action servers.

### 1.8 Required tf Transforms
This node does not require transforms.

### 1.9 Provided tf Transforms
This node does not provide transforms.

### 1.10 Bringup

```bash
roslaunch robotnik_joint_state_publisher joint_state_publisher.launch
```
