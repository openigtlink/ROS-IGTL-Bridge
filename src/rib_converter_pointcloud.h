/*=========================================================================

  Program:   Converter Class for PointCloud
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterPointCloud_H
#define __RIBConverterPointCloud_H

#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlpointcloud.h"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterPointCloud : public RIBConverter<ros_igtl_bridge::igtlpointcloud>
{

public:
  RIBConverterPointCloud();
  RIBConverterPointCloud(ros::NodeHandle *nh);
  RIBConverterPointCloud(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POLYDATA"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlpointcloud::ConstPtr & msg);
};


#endif // __Messageconverterpointcloud_H


