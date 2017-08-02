/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

// This Class adds a templated function (onROSMessagre) to
// the RIBConverterBase class.

#ifndef __RIBConverter_H
#define __RIBConverter_H

#include "ros/ros.h"

#include "rib_converter_base.h"
#include "ros/callback_queue.h"
#include "igtlMessageHeader.h"
//#include "igtlSocket.h"

template <typename MessageType>
class RIBConverter : public RIBConverterBase
{

public:
  RIBConverter();
  RIBConverter(ros::NodeHandle *nh);
  RIBConverter(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
protected:
  virtual void onROSMessage(const typename MessageType::ConstPtr& msg) = 0;

public:

  virtual bool publish(const char* topic);
  virtual bool subscribe(const char* topic);
  
protected:

  ~RIBConverter();

};

  
#include "rib_converter.tpp"


#endif // __RIBConverter_H
