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

//class RIBConverterPoint;
//class RIBConverterPointCloud;
//class RIBConverterTransform;
//class RIBConverterPolyData;
//class RIBConverterString;
//class RIBConverterImage;
//class RIBConverterVideo;
class RIBConverterBase;

class ROS_IGTL_Bridge
{
public:
  ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name);
  ~ROS_IGTL_Bridge();
  void Run();
  
  igtl::Socket::Pointer GetSocketPointer();
  virtual void CreateIGTLServer();
  virtual void ConnectToIGTLServer();

 protected:
  void AddConverter(RIBConverterBase* converter, uint32_t size, const char* topicPublish, const char* topicSubscribe);
  
private:
  igtl::Socket::Pointer socket;
  ros::NodeHandle *nh;

  //RIBConverterPoint* ribcpoint;
  //RIBConverterTransform* ribctransform;
  //RIBConverterPolyData* ribcpolydata;
  //RIBConverterPointCloud* ribcpointcloud;
  //RIBConverterString* ribcstring;
  //RIBConverterImage* ribcimage;
  std::vector< RIBConverterBase* > converters;
  
  // Callbacks
  virtual void IGTLReceiverThread();
  
};
#endif 
