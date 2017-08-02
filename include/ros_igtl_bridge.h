/*=========================================================================

  Program:   ROS-IGTL-Bridge Node
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef ROS_IGTL_BRIDGE_H
#define ROS_IGTL_BRIDGE_H

// ROS Includes
#include "ros/ros.h"

// OpenIGTLink Includes
#include "igtlServerSocket.h"

// C++ Includes
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <vector>

class RIBConverterManager;

class ROS_IGTL_Bridge
{
public:
  ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name);
  ~ROS_IGTL_Bridge();
  void Run();
  
  igtl::Socket::Pointer GetSocketPointer();

protected:

  virtual int StartIGTLServer();
  virtual int ConnectToIGTLServer();
  virtual void IGTLThread();

  igtl::Socket::Pointer socket;
  igtl::ServerSocket::Pointer serverSocket;
  igtl::ClientSocket::Pointer clientSocket;
    
  ros::NodeHandle *nh;
  RIBConverterManager * converterManager;
  
  bool isServer;   // Socket type for OpenIGTLink connection
  std::string address;
  int port;

};
#endif 
