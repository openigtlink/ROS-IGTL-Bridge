/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterBase_H
#define __MessageConverterBase_H

#include "ros/ros.h"
#include <ros/callback_queue.h>

template <typename MessageType>
class MessageConverterBase
{

public:
  MessageConverterBase();
  MessageConverterBase(ros::NodeHandle *nh);
  MessageConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return ""; };

public:
  virtual int onIGTLMessage(igtl::MessageHeader * header) = 0;
protected:
  virtual void onROSMessage(const typename MessageType::ConstPtr& msg) = 0;

public:
  void setNodeHandle(ros::NodeHandle* nh);
  void setTopicPublish(const char* topic);
  void setTopicSubscribe(const char* topic);
  void setQueueSize(uint32_t size) {this->queueSize = size; };

  virtual void setSocket(igtl::Socket * socket);

  bool start();
  
protected:

  ~MessageConverterBase();

  uint32_t    queueSize;
  
  std::string topicPublish;
  std::string topicSubscribe;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  ros::NodeHandle *nodeHandle;
  ros::SubscribeOptions options;
  
  igtl::Socket* socket;
  ros::CallbackQueue queue;
  
};

  
#include "message_converter_base.txx"


#endif // __MessageConverter_H
