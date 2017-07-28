/*=========================================================================

  Program:   Converter Class for String
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterString_H
#define __RIBConverterString_H

#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlstring.h"
#include "std_msgs/String.h"

// OpenIGTLink message files
#include "igtlMessageHeader.h"

class RIBConverterString : public RIBConverter<ros_igtl_bridge::igtlstring>
{

public:
  RIBConverterString();
  RIBConverterString(ros::NodeHandle *nh);
  RIBConverterString(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "STRING"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlstring::ConstPtr & msg);
};


#endif // __RIBConverterString_H


