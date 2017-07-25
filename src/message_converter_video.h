/*=========================================================================

  Program:   Converter Class for Video
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterVideo_H
#define __MessageConverterVideo_H

#include "ros/ros.h"
#include "message_converter_base.h"

class MessageConverterVideo : public MessageConverterBase<ros_igtl_bridge::igtlvideo>
{

public:
  MessageConverterVideo();
  MessageConverterVideo(ros::NodeHandle *nh);
  MessageConverterVideo(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "VIDEO"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlvideo::ConstPtr & msg);
};


MessageConverterVideo::MessageConverterVideo()
  : MessageConverterBase<ros_igtl_bridge::igtlvideo>()
{
}

MessageConverterVideo::MessageConverterVideo(ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlvideo>(nh)
{
}

MessageConverterVideo::MessageConverterVideo(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlvideo>(topicPublish, topicSubscribe, nh)
{
}

int MessageConverterVideo::onIGTLMessage(igtl::MessageHeader * header)
{
}

void MessageConverterVideo::onROSMessage(const ros_igtl_bridge::igtlvideo::ConstPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
  
  cv::Mat img; 
  cvtColor(cv_ptr->image,img, CV_RGB2GRAY);
  
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; 
  
  std_msgs::Header header; 
  header.stamp = ros::Time::now(); 
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
  img_bridge.toImageMsg(img_msg); 
  
  sensor_msgs::Image::Ptr msg = boost::make_shared<sensor_msgs::Image>(img_msg);
  
  /* debug window
     static const char WINDOW[] = "Image window";
     cv::namedWindow(WINDOW);
     cv::imshow(WINDOW, img);
     cv::waitKey(3);
  */
	
  int   size[]     = {msg->width,msg->height,1};       // image dimension
  float spacing[]  = {1,1,1};     // spacing (mm/pixel) 
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;
  
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetCoordinateSystem(2);
  imgMsg->SetDeviceName("ROS_IGTL_Bridge_Video");
  imgMsg->SetOrigin(0,0,0);
  imgMsg->AllocateScalars();
  //------------------------------------------------------------
  std::cout<< msg->data.size()<<std::endl;
  std::cout<< msg->step<<std::endl;
  std::cout<< msg->height<<std::endl;
  std::cout<< msg->width<<std::endl;
  
  image_pub.publish(msg);
  
  memcpy(imgMsg->GetScalarPointer(),(char*)(&msg->data[0]),msg->data.size());
  
  igtl::Matrix4x4 matrixa;
  igtl::IdentityMatrix(matrixa);
  matrixa[0][0] = -1;
  matrixa[1][1] = -1;
  
  imgMsg->SetMatrix(matrixa);
  
  //------------------------------------------------------------
  // Pack and send
  imgMsg->Pack();
  
  this->socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}



#endif // __MessageConverterVideo_H


