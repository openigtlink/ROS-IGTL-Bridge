/*=========================================================================

  Program:   ROS-IGTL-Bridge Node
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <boost/thread.hpp>

#include "ros_igtl_bridge.h"

#include "rib_converter_manager.h"
#include "rib_converter_point.h"
#include "rib_converter_pointcloud.h"
#include "rib_converter_transform.h"
#include "rib_converter_polydata.h"
#include "rib_converter_string.h"
#include "rib_converter_image.h"


//----------------------------------------------------------------------
ROS_IGTL_Bridge::ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name)
{
  ros::init(argc, argv, node_name);

  this->isServer = false;
  
  this->nh = new ros::NodeHandle;
  
  this->converterManager = new RIBConverterManager;
  this->converterManager->SetNodeHandle(this->nh);
  
  // Regisgter converter classes
  RIBConverterPoint * point = new RIBConverterPoint;
  RIBConverterTransform* transform = new RIBConverterTransform;
  RIBConverterPolyData* polydata = new RIBConverterPolyData;
  RIBConverterString* string = new RIBConverterString;
  RIBConverterImage* image = new RIBConverterImage;
  RIBConverterPointCloud* pointcloud = new RIBConverterPointCloud;
  
  this->converterManager->AddConverter(point, 10, "IGTL_POINT_IN", "IGTL_POINT_OUT");
  this->converterManager->AddConverter(transform, 10, "IGTL_TRANSFORM_IN", "IGTL_TRANSFORM_OUT");
  this->converterManager->AddConverter(polydata, 10, "IGTL_POLYDATA_IN", "IGTL_POLYDATA_OUT");
  this->converterManager->AddConverter(string, 10, "IGTL_STRING_IN", "IGTL_STRING_OUT");
  this->converterManager->AddConverter(image, 10, "IGTL_IMAGE_IN", "IGTL_IMAGE_OUT");
  this->converterManager->AddConverter(pointcloud, 10, "IGTL_POINTCLOUD_IN", "IGTL_POINTCLOUD_OUT");

  // run bridge as client or server
  std::string type;
  if(nh->getParam("/RIB_type",type))
    {
    if(type == "client")
      {
      //ConnectToIGTLServer();
      this->isServer = false;
      }
    else if(type == "server")
      {
      //StartIGTLServer();
      this->isServer = true;
      }
    else
      {
      ROS_ERROR("[ROS-IGTL-Bridge] Unknown Value for Parameter 'RIB_type'");
      ros::shutdown();
      }
    }
  else
    {
    short srvcl = 0;
    while(1)
      {
      std::cout << "[ROS-IGTL-Bridge] Please type <1> or <2> to run node as OpenIGTLink client or server"<<std::endl;
      std::cout << "1 : SERVER" << std::endl << "2 : CLIENT" << std::endl;
      std::cin>>srvcl;
      
      if (srvcl==1)
        {
        //StartIGTLServer();
        this->isServer = true;
        break;
        }
      else if (srvcl==2)
        {
        //ConnectToIGTLServer();
        this->isServer = false;
        break;
        }
      else
        {
        ROS_ERROR("[ROS-IGTL-Bridge] Invalid answer.");
        }
      }
    }

  this->port     = 18944;  // std port
  
  if (this->isServer)
    {
    if(this->nh->getParam("/RIB_port", this->port))
      {}
    else
      {
      ROS_INFO("[ROS-IGTL-Bridge] Input socket port: ");
      std::cin >> this->port;
      }
    }
  else // if this is a client
    {
    // get IP address
    if(nh->getParam("/RIB_server_ip", this->address))
      {}
    else
      {
      ROS_INFO("[ROS-IGTL-Bridge] Please enter ServerIP: ");
      std::cin >> this->address;
      }
    // get port
    if(nh->getParam("/RIB_port",this->port))
      {}
    else
      {
      ROS_INFO("[ROS-IGTL-Bridge] Please enter ServerPort:  ");
      std::cin >> this->port;
      }
    }

  //ROS_INFO("[ROS-IGTL-Bridge] ROS-IGTL-Bridge up and Running.");
  
  // start OpenIGTLink thread
  boost::thread* igtl_thread = new boost::thread(boost::bind(&ROS_IGTL_Bridge::IGTLThread, this));  

}

//----------------------------------------------------------------------
ROS_IGTL_Bridge::~ROS_IGTL_Bridge()
{
  //this->socket->CloseSocket();
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::Run()
{
  ros::spin();
}

//----------------------------------------------------------------------
igtl::Socket::Pointer ROS_IGTL_Bridge::GetSocketPointer()
{
  igtl::Socket::Pointer socket_ptr = static_cast<igtl::Socket::Pointer>(socket);
  return socket_ptr;
}


//----------------------------------------------------------------------
int ROS_IGTL_Bridge::StartIGTLServer()
{
  static igtl::ServerSocket::Pointer serverSocket;

  if (serverSocket.IsNull()) // if called for the first time
    {
    serverSocket = igtl::ServerSocket::New();
    int c = serverSocket->CreateServer(this->port);
    if (c < 0)
      {
      ROS_ERROR("[ROS-IGTL-Bridge] Cannot create a server socket.");
      return 0;
      }
    }
  
  ROS_INFO("[ROS-IGTL-Bridge] Server socket created. Please connect to port: %d",port);
  
  // wait for connection
  while (1)
    {
    this->socket = serverSocket->WaitForConnection(1000);
    this->converterManager->SetSocket(this->socket);
    if (this->socket.IsNotNull()) 
      {   
      return 1;
      }
    }
  
  return 0;
}

//----------------------------------
int ROS_IGTL_Bridge::ConnectToIGTLServer()
{
  static igtl::ClientSocket::Pointer clientSocket;

  if (clientSocket.IsNull()) // if called for the first time
    {
    clientSocket = igtl::ClientSocket::New();
    }
  
  // connect to server
  int r = clientSocket->ConnectToServer(this->address.c_str(), this->port);
  
  if (r != 0)
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Cannot connect to server.");
    return 0;
    }
  
  this->socket = (igtl::Socket *)(clientSocket);
  this->converterManager->SetSocket(this->socket);
  return 1;
}

void ROS_IGTL_Bridge::IGTLThread()
{
  int r;

  while (1)
    {
    if (this->isServer)
      {
      r = this->StartIGTLServer();
      }
    else
      {
      r = this->ConnectToIGTLServer();
      }
    if (r == 0)
      {
      ros::shutdown();  // TODO: Can the thread shutdown the process?
      }
      
    igtl::MessageHeader::Pointer headerMsg;
    headerMsg = igtl::MessageHeader::New();
    int rs = 0;
    int loop = 1;
    
    while(loop)
      {
      headerMsg->InitPack();
      // receive packet
      bool timeout(false);
      rs = this->socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize(), timeout);
          
      if (rs == 0)
        {
        this->socket->CloseSocket();
        this->converterManager->SetSocket(NULL);
        loop = 0; // Terminate the thread.
        }
          
      if (rs != headerMsg->GetPackSize())
        continue;
          
      this->converterManager->ProcessIGTLMessage(headerMsg);
      }
    }
}

