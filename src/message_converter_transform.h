/*=========================================================================

  Program:   Converter Class for Transform
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __MessageConverterTransform_H
#define __MessageConverterTransform_H

#include "message_converter_base.h"

// ROS header files
#include "ros/ros.h"

// ROS message header files
#include "ros_igtl_bridge/igtltransform.h"
#include "geometry_msgs/Transform.h"


// OpenIGTLink message files
#include "igtlTransformMessage.h"

class MessageConverterTransform : public MessageConverterBase<ros_igtl_bridge::igtltransform>
{

public:
  MessageConverterTransform();
  MessageConverterTransform(ros::NodeHandle *nh);
  MessageConverterTransform(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh=NULL);
  
  virtual uint32_t queueSizePublish() { return 10; }
  virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "TRANSFORM"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros_igtl_bridge::igtltransform::ConstPtr & msg);
};


MessageConverterTransform::MessageConverterTransform()
  : MessageConverterBase<ros_igtl_bridge::igtltransform>()
{
}

MessageConverterTransform::MessageConverterTransform(ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtltransform>(nh)
{
}

MessageConverterTransform::MessageConverterTransform(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : MessageConverterBase<ros_igtl_bridge::igtltransform>(topicPublish, topicSubscribe, nh)
{
}

int MessageConverterTransform::onIGTLMessage(igtl::MessageHeader * header)
{
  // create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();
  
  // receive transform data from the socket
  this->socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
  
  // unpack message
  int c = transMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) 
    { 
    // retrive the transform data
    ros_igtl_bridge::igtltransform msg;
    igtl::Matrix4x4 igtlmatrix;
    igtl::IdentityMatrix(igtlmatrix);
    transMsg->GetMatrix(igtlmatrix);
    
    msg.transform.translation.x = igtlmatrix[0][3];
    msg.transform.translation.y = igtlmatrix[1][3];
    msg.transform.translation.z = igtlmatrix[2][3];
    
    float quaternion [4];
    igtl::MatrixToQuaternion(igtlmatrix,quaternion);
    

    msg.transform.rotation.x = quaternion[0];
    msg.transform.rotation.y = quaternion[1];
    msg.transform.rotation.z = quaternion[2];
    msg.transform.rotation.w = quaternion[3];
    
    msg.name = transMsg->GetDeviceName();
    // publish to topic
    this->publisher.publish(msg);
    return 1;
    }
  else 
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: TRANSFORM.");
    return 0;
    }
}

void MessageConverterTransform::onROSMessage(const ros_igtl_bridge::igtltransform::ConstPtr & msg)
{
  // convert msg to igtl matrix
  igtl::Matrix4x4 sendMatrix;
  igtl::IdentityMatrix(sendMatrix);
  
  // rotation
  float quaternion [4];
  quaternion[0] = msg->transform.rotation.x;
  quaternion[1] = msg->transform.rotation.y;
  quaternion[2] = msg->transform.rotation.z;
  quaternion[3] = msg->transform.rotation.w;
  igtl::QuaternionToMatrix(quaternion,sendMatrix);
  
  // translation
  sendMatrix[0][3] = msg->transform.translation.x;
  sendMatrix[1][3] = msg->transform.translation.y;
  sendMatrix[2][3] = msg->transform.translation.z;
  
  // generate message
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetDeviceName(msg->name.c_str());
  transMsg->SetMatrix(sendMatrix);
  transMsg->Pack();
  this->socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
}


#endif // __MessageConverterTransform_H


