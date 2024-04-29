#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot/odom", "robot/base_link"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf_publisher");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/robot/robotnik_base_hw/odom", 10, &odomCallback);

    ros::spin();
    return 0;
}