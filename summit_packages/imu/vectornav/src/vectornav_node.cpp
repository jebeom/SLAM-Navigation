#include <ros/ros.h>
#include <vectornav/vectornav.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "vectornav_node");
  ros::NodeHandle nh, pnh{"~"};
  vectornav::Vectornav vectornav(nh, pnh);
  vectornav.spin();
  return 0;
}