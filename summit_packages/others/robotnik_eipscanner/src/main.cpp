#include <robotnik_eipscanner/eipscanner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eipscanner_node");
  ros::NodeHandle n;

  robotnik_eipscanner::EIPScanner publisher(n);
  publisher.start();
}
