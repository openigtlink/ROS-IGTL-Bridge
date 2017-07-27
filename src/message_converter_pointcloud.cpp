/*=========================================================================

  Program:   Converter Class for Point Cloud
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "message_converter_pointcloud.h"
#include "ros/ros.h"
#include "igtlPointMessage.h"


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





