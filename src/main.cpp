#include <ros_igtl_bridge.h>
int main (int argc, char *argv[])
{
  ROS_IGTL_Bridge bridge_node(argc, argv, "ros_igtl_bridge_node");
  bridge_node.Run();
  
  return 0;
}
