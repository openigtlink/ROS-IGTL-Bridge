/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterPoint_TXX
#define __MessageConverterPoint_TXX

#include "boost/bind.hpp"

template <typename MessageType>
MessageConverterBase<MessageType>::MessageConverterBase()
{
  this->nodeHandle = NULL;
}

template <typename MessageType>
MessageConverterBase<MessageType>::MessageConverterBase(ros::NodeHandle *nh)
{
  this->setNodeHandle(nh);
}

template <typename MessageType>
MessageConverterBase<MessageType>::MessageConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
{
  this->topicPublish = topicPublish;
  this->topicSubscribe = topicSubscribe;
  this->setNodeHandle(nh);
}

template <typename MessageType>
MessageConverterBase<MessageType>::~MessageConverterBase()
{
}

template <typename MessageType>
void MessageConverterBase<MessageType>::setNodeHandle(ros::NodeHandle* nh)
{
  this->nodeHandle = nh;
}


template <typename MessageType>
bool MessageConverterBase<MessageType>::start()
{
  if (!this->nodeHandle)
    {
      return false;
    }
  
  // TODO: Queue size (second argument) should be configurable
  this->publisher = nodeHandle->advertise<MessageType>(this->topicPublish, 10);
  std::cerr << "TOPIC: " << this->topicSubscribe << std::endl;
  
  this->options =
    ros::SubscribeOptions::create<MessageType>(this->topicSubscribe,
					       10, // queue length
					       boost::bind(&MessageConverterBase<MessageType>::onROSMessage, this, _1),
					       ros::VoidPtr(), // tracked object, we don't need one thus NULL
					       &this->queue // pointer to callback queue object
					       );
  
  this->subscriber = nodeHandle->subscribe(options);
  return true;
}

template <typename MessageType>
void MessageConverterBase<MessageType>::setTopicPublish(const char* topic)
{
  this->topicPublish = topic;
}

template <typename MessageType>
void MessageConverterBase<MessageType>::setTopicSubscribe(const char* topic)
{
  this->topicSubscribe = topic;
}

template <typename MessageType>
void MessageConverterBase<MessageType>::setSocket(igtl::Socket * socket)
{
  this->socket = socket;
}

#endif // __MessageConverterPoint_TXX

