/*=========================================================================

  Program:   Converter Class for Polydata
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterPolyData_H
#define __MessageConverterPolyData_H

#include "message_converter_base.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtlpolydata.h"

// VTK header files
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

// OpenIGTLink message files
#include "igtlMessageHeader.h"


class MessageConverterPolyData : public MessageConverterBase<ros_igtl_bridge::igtlpolydata>
{

public:
  MessageConverterPolyData();
  MessageConverterPolyData(ros::NodeHandle *nh);
  MessageConverterPolyData(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POLYDATA"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtlpolydata::ConstPtr & msg);
  
protected:
  void msgToPolyData(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg, vtkSmartPointer<vtkPolyData> polydata);
  ros_igtl_bridge::igtlpolydata polyDataToMsg(const char* name, vtkSmartPointer<vtkPolyData> polydata);
};


#endif // __MessageConverterPolyData_H


