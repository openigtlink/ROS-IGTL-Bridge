#include <ROS_IGTL_Test.h>


int main (int argc, char *argv[])
{
  ROS_IGTL_Test test_node(argc, argv, "ROS_IGTL_Test");
  test_node.Run();

  return 0;
}
