#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ur_gripper_hw_interface/ur_gripper_hw_interface.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_gripper_hw_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    ur_gripper_hw_interface::UrGripperHwInterface rhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
   
    return 0;
}