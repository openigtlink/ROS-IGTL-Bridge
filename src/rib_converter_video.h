/*=========================================================================

  Program:   Converter Class for Video
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterVideo_H
#define __RIBConverterVideo_H

#include "rib_converter.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlvidel.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

// OpenCV header files
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenIGTLink message files
#include "igtlMessageHeader.h"


class RIBConverterVideo : public RIBConverter<ros_igtl_bridge::igtlvideo>
{

public:
  RIBConverterVideo();
  RIBConverterVideo(ros::NodeHandle *nh);
  RIBConverterVideo(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "VIDEO"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlvideo::ConstPtr & msg);
};


#endif // __RIBConverterVideo_H


