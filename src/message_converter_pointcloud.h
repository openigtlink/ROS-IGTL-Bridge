/*=========================================================================

  Program:   Converter Class for PointCloud
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterPointCloud_H
#define __MessageConverterPointCloud_H

#include "message_converter_base.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlpointcloud.h"

// OpenIGTLink message files


class MessageConverterPointCloud : public MessageConverterBase<ros_igtl_bridge::igtlpointcloud>
{

public:
  MessageConverterPointCloud();
  MessageConverterPointCloud(ros::NodeHandle *nh);
  MessageConverterPointCloud(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POLYDATA"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlpointcloud::ConstPtr & msg);
};


MessageConverterPointCloud::MessageConverterPointCloud()
  : MessageConverterBase<ros_igtl_bridge::igtlpointcloud>()
{
}

MessageConverterPointCloud::MessageConverterPointCloud(ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlpointcloud>(nh)
{
}

MessageConverterPointCloud::MessageConverterPointCloud(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlpointcloud>(topicPublish, topicSubscribe, nh)
{
}

int MessageConverterPointCloud::onIGTLMessage(igtl::MessageHeader * header)
{
  
}

void MessageConverterPointCloud::onROSMessage(const ros_igtl_bridge::igtlpointcloud::ConstPtr & msg)
{
  
  int pcl_size = msg->pointdata.size();
  if (!pcl_size) 
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Pointcloud is empty!");
    return;
    }
  
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());
  
  for (int i = 0; i<pcl_size;i++)
    {
    igtl::PointElement::Pointer pointE; 
    pointE = igtl::PointElement::New();
    pointE->SetPosition(msg->pointdata[i].x, msg->pointdata[i].y,msg->pointdata[i].z);		
    pointMsg->AddPointElement(pointE);
    }
  pointMsg->Pack();
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());

}


#endif // __Messageconverterpointcloud_H


