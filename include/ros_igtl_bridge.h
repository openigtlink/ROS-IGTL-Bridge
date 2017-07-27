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


class MessageConverterPoint;
class MessageConverterPointCloud;
class MessageConverterTransform;
class MessageConverterPolyData;
class MessageConverterString;
class MessageConverterImage;
class MessageConverterVideo;


class ROS_IGTL_Bridge
{
public:
  ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name);
  ~ROS_IGTL_Bridge();
  void Run();
  
  igtl::Socket::Pointer socket;
  igtl::Socket::Pointer GetSocketPointer();
  virtual void CreateIGTLServer();
  virtual void ConnectToIGTLServer();
  
private:
  ros::NodeHandle *nh;

  MessageConverterPoint* mcpoint;
  MessageConverterTransform* mctransform;
  MessageConverterPolyData* mcpolydata;
  MessageConverterPointCloud* mcpointcloud;
  MessageConverterString* mcstring;
  MessageConverterImage* mcimage;
  
  // Callbacks
  virtual void IGTLReceiverThread();
  
};
#endif 
