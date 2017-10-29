/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterBase_H
#define __RIBConverterBase_H

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "igtlMessageHeader.h"
#include "igtlSocket.h"

class RIBConverterManager;

class RIBConverterBase
{

public:
  RIBConverterBase();
  RIBConverterBase(ros::NodeHandle *nh);
  RIBConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return ""; };

public:
  virtual int onIGTLMessage(igtl::MessageHeader * header) = 0;
protected:
  //virtual void onROSMessage(const typename MessageType::ConstPtr& msg) = 0;

public:
  void setNodeHandle(ros::NodeHandle* nh) { this->nodeHandle = nh; }
  void setQueue(ros::NodeHandle* nh) { this->queue = nh->getCallbackQueue(); }
  //void setSocket(igtl::Socket * socket) { this->socket = socket; }
  void setQueueSize(uint32_t size) { this->queueSize = size; }
  void setup(ros::NodeHandle* nh, uint32_t queuSize);
  void setManager(RIBConverterManager* manager) { this->manager = manager; }

  virtual bool publish(const char* topic) = 0;
  virtual bool subscribe(const char* topic) = 0;
  
  
protected:

  ~RIBConverterBase();

  uint32_t    queueSize;
  
  std::string topicPublish;
  std::string topicSubscribe;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  ros::NodeHandle *nodeHandle;
  ros::SubscribeOptions options;
  
  ros::CallbackQueueInterface* queue;

  RIBConverterManager* manager;
  
};

#endif // __RIBConverter_H
