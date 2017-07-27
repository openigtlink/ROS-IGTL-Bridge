/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "message_converter_image.h"
#include "ros/ros.h"
#include "igtlMessageHeader.h"
#include "igtlImageMessage.h"

MessageConverterImage::MessageConverterImage()
  : MessageConverterBase<ros_igtl_bridge::igtlimage>()
{
}

MessageConverterImage::MessageConverterImage(ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlimage>(nh)
{
}

MessageConverterImage::MessageConverterImage(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlimage>(topicPublish, topicSubscribe, nh)
{
}

int MessageConverterImage::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetMessageHeader(header);
  imgMsg->AllocatePack();

  this->socket->Receive(imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());
  int c = imgMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: IMAGE.");
    return 0;
    }
  
  std::vector<uint8_t> image;
  sensor_msgs::ImagePtr img_msg  (new sensor_msgs::Image()) ;
  
  float spacing[] = {0,0,0};
  imgMsg->GetSpacing(spacing);
  
  int   imgsize[] = {0,0,0};
  imgMsg->GetDimensions(imgsize);
  img_msg->height = imgsize[1];
  img_msg->width = imgsize[0];
  
  ROS_INFO("h %d",img_msg->height);
  ROS_INFO("w %d",img_msg->width);
  
  img_msg->encoding = "mono8";
  img_msg->is_bigendian = false;
  img_msg->step = imgsize[0];
  ROS_INFO("s %d",img_msg->step);
  size_t size = img_msg->step* img_msg->height;
  img_msg->data.resize(size);
  
  memcpy((char*)(&img_msg->data[0]),imgMsg->GetScalarPointer(),size); 
  
  this->publisher.publish(img_msg);

  return 1;

}

void MessageConverterImage::onROSMessage(const ros_igtl_bridge::igtlimage::ConstPtr & msg)
{
  int   size[]     = {msg->z_steps,msg->y_steps,msg->x_steps};       // image dimension
  float spacing[]  = {msg->z_spacing,msg->y_spacing,msg->x_spacing};     // spacing (mm/pixel) 
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;
  
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetCoordinateSystem(2);
  imgMsg->SetDeviceName(msg->name.c_str());
  imgMsg->SetOrigin(0,0,0);
  imgMsg->AllocateScalars();
  //-----------------------------------------------------------
  
  memcpy(imgMsg->GetScalarPointer(),(char*)(&msg->data[0]),msg->data.size());
  
  igtl::Matrix4x4 matrixa;
  igtl::IdentityMatrix(matrixa);
  imgMsg->SetMatrix(matrixa);
  
  //------------------------------------------------------------
  // Pack and send
  imgMsg->Pack();
  
  this->socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}



