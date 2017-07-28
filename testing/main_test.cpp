/*=========================================================================

  Program:   ROS-IGTL-Bridge Test Server
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <ros_igtl_test.h>


int main (int argc, char *argv[])
{
  ROS_IGTL_Test test_node(argc, argv, "ros_igtl_bridge_test");
  test_node.Run();

  return 0;
}
