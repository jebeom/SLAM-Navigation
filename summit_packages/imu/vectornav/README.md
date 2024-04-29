# Vectornav ROS Driver

This repository contains the ROS packages for the Vectornav INS & GPS devices.

> **_NOTE:_**  Currently only Vectornav VN100 with version `2.1.0` and `3.0.0` has been tested. Please open an issue if you need support for other versions.

## Usage

### Parameters

The following parameters for `vnpub` node:

Required:
* `~serial_port` (`string`, required): The device name of the Vectornav device. For example, `/dev/ttyUSB_IMU`.
* `~serial_baud` (`int`, required): The baudrate of the Vectornav device. For example, `115200`.
* `~frame_id` (`string`, required): The frame id of the Vectornav device. For example, `robot_imu_link`.


Optional:
* `~acc_bias_enable` (`bool`, default: `false`): Enable service to calibrate accelerometer bias.
* `~set_acc_bias_seconds` (`double`, default: `2.5`): Time in seconds to take samples for accelerometer bias calibration.
* `~async_output_rate` (`int`, default: `200`): The rate of the async output. For example, `200`.
* `~fixed_imu_rate` (`int`, default: `800`): The fixed rate of the IMU data. For example, `800`.
* `~adjust_ros_timestamp` (`bool`, default: `true`): Use sensor timestamp instead of ROS timestamp to mitigate USB communication issues.
* `~map_frame_id` (`string`, default: `map`): The map frame id of the Vectornav device. For example, `map`.

* `~linear_accel_covariance` (`double[9]`, default: `[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]`): The covariance of the linear acceleration.
* `~angular_vel_covariance` (`double[9]`, default: `[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]`): The covariance of the angular velocity.
* `~orientation_covariance` (`double[9]`, default: `[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]`): The covariance of the orientation.

* `~rotation_reference_frame` (`double[9]`, default: `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 1.0]`): The rotation reference frame of the Vectornav device. For example, `[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 1.0]` for NED frame.

### Topics

The following topics are published by the `vnpub` node:
* `~data` (`sensor_msgs/Imu`): The IMU data with standard ROS messages.
* `~mag` (`sensor_msgs/MagneticField`): The magnetic field data with standard ROS messages.
* `~gps` (`sensor_msgs/NavSatFix`): The GPS data with standard ROS messages.
* `~odom` (`nav_msgs/Odometry`): The odometry data with standard ROS messages.
* `~temperature` (`sensor_msgs/Temperature`): The temperature data with standard ROS messages.
* `~atm_pressure` (`sensor_msgs/FluidPressure`): The atmospheric pressure data with standard ROS messages.
* `~ins_status` (`vectornav_msgs/InsStatus`): The INS status data with custom ROS messages.

### Services

The following services are provided by the `vnpub` node:
* `~set_acc_bias` (`vectornav_msgs/SetAccBias`): Set the accelerometer bias to `[0.0, 0.0, 9.81]` with mean of samples taken during `set_acc_bias_seconds` seconds.
* `~reset_acc_bias` (`std_srvs/Empty`): Reset the accelerometer bias to zero.

> **_NOTE:_** The services are only available if `acc_bias_enable` is set to `true`.

## Code of Conduct
This project has adopted the [Robotnik Code of Conduct](https://to.do/code_of_conduct_faq). For more information see the [Code of Conduct FAQ](https://to.do/code_of_conduct_faq) or contact [opencode@robotnik.es](opencode@robotnik.es) with any additional questions or comments.

## License
Copyright (c) 2023, Robotnik Automation S.L. All rights reserved.

The original code under MIT License can be found on this [repository](https://github.com/dawonn/vectornav).
This repository has been partitioned from another project, originally licensed under the MIT License. However, the current project is licensed under the BSD 2-Clause License.

The original code can be found [here](https://github.com/dawonn/vectornav).

[//]: # (Links)
