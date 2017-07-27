/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterPoint_TXX
#define __RIBConverterPoint_TXX

#include <boost/bind.hpp>

template <typename MessageType>
RIBConverterBase<MessageType>::RIBConverterBase()
{
  this->nodeHandle = NULL;
}

template <typename MessageType>
RIBConverterBase<MessageType>::RIBConverterBase(ros::NodeHandle *nh)
{
  this->setNodeHandle(nh);
}

template <typename MessageType>
RIBConverterBase<MessageType>::RIBConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
{
  this->topicPublish = topicPublish;
  this->topicSubscribe = topicSubscribe;
  this->setNodeHandle(nh);
}

template <typename MessageType>
RIBConverterBase<MessageType>::~RIBConverterBase()
{
}

template <typename MessageType>
bool RIBConverterBase<MessageType>::publish(const char* topic)
{
  if (!this->nodeHandle)
    {
      return false;
    }
  
  if (!topic)
    {
      this->topicPublish = topic;
    }
  
  // TODO: Queue size (second argument) should be configurable
  this->publisher = nodeHandle->advertise<MessageType>(this->topicPublish, this->queueSize);
  std::cerr << "TOPIC: " << this->topicSubscribe << std::endl;
  
  return true;
}

  
template <typename MessageType>
bool RIBConverterBase<MessageType>::subscribe(const char* topic)
{
  if (!this->nodeHandle)
    {
      return false;
    }
  
  if (!topic)
    {
      this->topicSubscribe = topic;
    }

  this->options =
    ros::SubscribeOptions::create<MessageType>(
       this->topicSubscribe,
       this->queueSize, // queue length
       boost::bind(&RIBConverterBase<MessageType>::onROSMessage, this, _1),
       ros::VoidPtr(), // tracked object, we don't need one thus NULL
       &this->queue // pointer to callback queue object
      );
  this->subscriber = this->nodeHandle->subscribe(options);

  return true;
}

template <typename MessageType>
void RIBConverterBase<MessageType>::setup(ros::NodeHandle* nh, igtl::Socket * socket, uint32_t queuSize)
{
  this->setNodeHandle(nh);
  this->setSocket(socket);
  this->setQueueSize(queueSize);
}

//template <typename MessageType>
//void RIBConverterBase<MessageType>::setup(ros::NodeHandle* nh, igtl::Socket * socket, const char* topicSubscribe, const char* topicPublish)
//{
//  this->nodeHandle = nh;
//  this->socket = socket;
//  this->topicSubscribe = topicSubscribe;
//  this->topicPublish = topicPublish;
//}

#endif // __RIBConverterPoint_TXX

