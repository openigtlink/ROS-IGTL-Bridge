/*=========================================================================

  Program:   Converter Class for Transform
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterTransform_H
#define __RIBConverterTransform_H

#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtltransform.h"
#include "geometry_msgs/Transform.h"


// OpenIGTLink message files
#include "igtlTransformMessage.h"

class RIBConverterTransform : public RIBConverter<ros_igtl_bridge::igtltransform>
{

public:
  RIBConverterTransform();
  RIBConverterTransform(ros::NodeHandle *nh);
  RIBConverterTransform(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "TRANSFORM"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtltransform::ConstPtr & msg);
};


#endif // __RIBConverterTransform_H


