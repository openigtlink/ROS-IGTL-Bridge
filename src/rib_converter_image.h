/*=========================================================================

  Program:   Converter Class for Image
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterImage_H
#define __RIBConverterImage_H

#include "rib_converter.h"

// ROS header files
#include "sensor_msgs/Image.h"

// ROS message header files
#include "ros_igtl_bridge/igtlimage.h"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterImage : public RIBConverter<ros_igtl_bridge::igtlimage>
{

public:
  RIBConverterImage();
  RIBConverterImage(ros::NodeHandle *nh);
  RIBConverterImage(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "IMAGE"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlimage::ConstPtr & msg);
};


#endif // __RIBConverterImage_H


