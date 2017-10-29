/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_base.h"


RIBConverterBase::RIBConverterBase()
{
  this->nodeHandle = NULL;
}

RIBConverterBase::RIBConverterBase(ros::NodeHandle *nh)
{
  this->setNodeHandle(nh);
}

RIBConverterBase::RIBConverterBase(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
{
  this->topicPublish = topicPublish;
  this->topicSubscribe = topicSubscribe;
  this->setNodeHandle(nh);
}

RIBConverterBase::~RIBConverterBase()
{
}


void RIBConverterBase::setup(ros::NodeHandle* nh, uint32_t queuSize)
{
  this->setNodeHandle(nh);
  this->setQueue(nh);
  this->setQueueSize(queueSize);
}

//void RIBConverterBase::setup(ros::NodeHandle* nh, igtl::Socket * socket, const char* topicSubscribe, const char* topicPublish)
//{
//  this->nodeHandle = nh;
//  this->socket = socket;
//  this->topicSubscribe = topicSubscribe;
//  this->topicPublish = topicPublish;
//}


