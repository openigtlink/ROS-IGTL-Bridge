/*=========================================================================

  Program:   Converter Class for Transform
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_transform.h"
#include "rib_converter_manager.h"
#include "ros/ros.h"
#include "igtlTransformMessage.h"

RIBConverterTransform::RIBConverterTransform()
  : RIBConverter<ros_igtl_bridge::igtltransform>()
{
}

RIBConverterTransform::RIBConverterTransform(ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtltransform>(nh)
{
}

RIBConverterTransform::RIBConverterTransform(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtltransform>(topicPublish, topicSubscribe, nh)
{
}

int RIBConverterTransform::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
      return 0;
    }

  // create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();

  // receive transform data from the socket
  bool timeout(false);
  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize(), timeout);

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

void RIBConverterTransform::onROSMessage(const ros_igtl_bridge::igtltransform::ConstPtr & msg)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
      return;
    }

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
  
  socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
  
}


