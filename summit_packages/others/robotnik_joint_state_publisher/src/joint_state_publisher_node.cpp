#include <robotnik_joint_state_publisher/JointStatePublisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_state_publisher_node");
  ros::NodeHandle n;

  JointStatePublisher publisher(n);
  publisher.start();
}
