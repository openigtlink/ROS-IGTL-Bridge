/*=========================================================================

  Program:   ROS-IGTL-Bridge Node
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "ros_igtl_bridge.h"

int main (int argc, char *argv[])
{
  ROS_IGTL_Bridge bridge_node(argc, argv, "ros_igtl_bridge_node");
  bridge_node.Run();
  
  return 0;
}
