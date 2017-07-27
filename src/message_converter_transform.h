/*=========================================================================

  Program:   Converter Class for Transform
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterTransform_H
#define __MessageConverterTransform_H

#include "message_converter_base.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtltransform.h"
#include "geometry_msgs/Transform.h"


// OpenIGTLink message files
#include "igtlTransformMessage.h"

class MessageConverterTransform : public MessageConverterBase<ros_igtl_bridge::igtltransform>
{

public:
  MessageConverterTransform();
  MessageConverterTransform(ros::NodeHandle *nh);
  MessageConverterTransform(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "TRANSFORM"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtltransform::ConstPtr & msg);
};


#endif // __MessageConverterTransform_H


