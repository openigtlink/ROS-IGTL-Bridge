#include <ROS_IGTL_Bridge.h>
int main (int argc, char *argv[])
{
    ROS_IGTL_Bridge bridge_node(argc, argv, "ROS_IGTL_Bridge");
    bridge_node.Run();

    return 0;
}
