/*=========================================================================

  Program:   Converter Class for Point
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterPoint_H
#define __RIBConverterPoint_H

#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlpoint.h"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterPoint : public RIBConverter<ros_igtl_bridge::igtlpoint>
{

public:
  RIBConverterPoint();
  RIBConverterPoint(ros::NodeHandle *nh);
  RIBConverterPoint(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POINT"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlpoint::ConstPtr & msg);
};

#endif // __RIBConverterPoint_H


