/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterPoint_H
#define __MessageConverterPoint_H

#include "ros/ros.h"
#include "message_converter_base.h"

class MessageConverterPoint : public MessageConverterBase<ros_igtl_bridge::igtlpoint>
{

public:
  MessageConverterPoint() {};
  MessageConverterPoint(ros::NodeHandle *nh);
  MessageConverterPoint(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  uint32_t queueSizePublish() { return 10; }
  uint32_t queueSizeSubscribe() { return 10; }
  const char* defaultTopicPublish() { return "IGTL_POINT_IN"; }
  const char* defaultTopicSubscribe() { return "IGTL_POINT_OUT"; }

  virtual const char* messageTypeString() { return "POINT"; }
  virtual int onReceiveIGTLMessage(igtl::MessageHeader * header);

 protected:

  virtual void callback(const ros_igtl_bridge::igtlpoint::ConstPtr & msg);
};

MessageConverterPoint::MessageConverterPoint(ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlpoint>(nh)
{
}

MessageConverterPoint::MessageConverterPoint(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtlpoint>(topicPublish, topicSubscribe, nh)
{
}

int MessageConverterPoint::onReceiveIGTLMessage(igtl::MessageHeader * header)
{
}

void MessageConverterPoint::callback(const ros_igtl_bridge::igtlpoint::ConstPtr & msg)
{
  geometry_msgs::Point point = msg->pointdata;
  
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());
  
  igtl::PointElement::Pointer pointE; 
  pointE = igtl::PointElement::New();
  pointE->SetPosition(point.x, point.y,point.z);
  
  pointMsg->AddPointElement(pointE);
  
  pointMsg->Pack();
  
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());

}


#endif // __MessageConverterPoint_H


