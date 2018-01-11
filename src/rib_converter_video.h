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
//#include "ros_igtl_bridge/igtlvideo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

// OpenCV header files
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenIGTLink message files
#include "igtlMessageHeader.h"
#include "igtlVideoMessage.h"

#if defined(OpenIGTLink_USE_VP9)
#include "VP9Encoder.h"
#include "VP9Decoder.h"
#endif


class RIBConverterVideo : public RIBConverter<sensor_msgs::Image>
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
  
  virtual void InitCodec();
  virtual void onROSMessage(const sensor_msgs::Image::ConstPtr & msg);

protected:
  GenericEncoder* VideoEncoder;
  GenericDecoder* VideoDecoder;
  unsigned char* ImageBuffer;
  
};


#endif // __RIBConverterVideo_H


