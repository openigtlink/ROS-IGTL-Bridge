/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverter_H
#define __MessageConverter_H

#include "ros/ros.h"

template <typename MessageType>
class MessageConverterBase
{

public:
  MessageConverterBase();
  MessageConverterBase(ros::NodeHandle *nh);
  MessageConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);

  virtual void setNodeHandle(ros::NodeHandle* nh);
  virtual void setTopicPublish(const char* topic);
  virtual void setTopicSubscribe(const char* topic);

  virtual void setSocket(igtl::Socket * socket);

  bool start();

  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* defaultTopicPublish() { return "IGTL_POINT_IN"; }
  virtual const char* defaultTopicSubscribe() { return "IGTL_POINT_OUT"; }

  virtual const char* messageTypeString() { return "POINT"; }
  virtual int onReceiveIGTLMessage(igtl::MessageHeader * header) = 0;
  
protected:

  ~MessageConverterBase();

  std::string topicPublish;
  std::string topicSubscribe;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  ros::NodeHandle *nodeHandle;
  ros::SubscribeOptions options;
  
  igtl::Socket* socket;
  
  virtual void callback(const typename MessageType::ConstPtr& msg);
  
  
};

  
#include "message_converter_base.txx"


#endif // __MessageConverter_H
