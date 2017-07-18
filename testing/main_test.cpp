#include <ros_igtl_test.h>


int main (int argc, char *argv[])
{
  ROS_IGTL_Test test_node(argc, argv, "ros_igtl_bridge_test");
  test_node.Run();

  return 0;
}
