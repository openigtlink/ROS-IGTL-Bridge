/*=========================================================================

  Program:   Converter Manager
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more converter_managerrmation.

=========================================================================*/

// This class is used to share the information 

#ifndef __rib_converter_manager_h
#define __rib_converter_manager_h

#include "ros/ros.h"
#include "igtlSocket.h"
#include "igtlMessageHeader.h"

class RIBConverterBase;

class RIBConverterManager
{
 public:
  RIBConverterManager();

  void SetSocket(igtl::Socket* socket) { this->socket = socket; };
  igtl::Socket::Pointer GetSocket();
  
  void SetNodeHandle(ros::NodeHandle *nh) { this->nh = nh; };
  
  void AddConverter(RIBConverterBase* converter, uint32_t size, const char* topicPublish, const char* topicSubscribe);

  // TODO: RemoveConverter()

  void ProcessIGTLMessage(igtl::MessageHeader* headerMsg);
  
 protected:
  ~RIBConverterManager();
  
  igtl::Socket::Pointer socket;
  ros::NodeHandle *nh;
  std::vector< RIBConverterBase* > converters;
};

#endif // __rib_converter_manager_h
