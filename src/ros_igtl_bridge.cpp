#include <boost/thread.hpp>

#include "eros_igtl_bridge.h"
#include "ShowPolyData.h"

#include "message_converter_point.h"
#include "message_converter_pointcloud.h"
#include "message_converter_transform.h"
#include "message_converter_polydata.h"
#include "message_converter_string.h"
#include "message_converter_image.h"


//----------------------------------------------------------------------
ROS_IGTL_Bridge::ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name)
{
  ros::init(argc, argv, node_name);
  nh = new ros::NodeHandle;	
  
  // run bridge as client or server
  std::string type;
  ROS_INFO("[ROS-IGTL-Bridge] a");
  if(nh->getParam("/RIB_type",type))
    {
    ROS_INFO("[ROS-IGTL-Bridge] b ");
    if(type == "client")
      ConnectToIGTLServer();		
    else if(type == "server")
      CreateIGTLServer();
    else
      ROS_ERROR("[ROS-IGTL-Bridge] Unknown Value for Parameter 'RIB_type'");	
    }
  else
    {
    short srvcl = 0;
    while(1)
      {
      std::cout<<"[ROS-IGTL-Bridge] Please type <1> or <2> to run node as OpenIGTLink client or server"<<std::endl<<"1 : SERVER"<<std::endl<<"2 : CLIENT"<<std::endl;
      std::cin>>srvcl;
      
      if (srvcl==1)
        {
        CreateIGTLServer();
        break;
        }
      else if (srvcl==2)
        {
        ConnectToIGTLServer();
        break;
        }
      }
    }
  
  ROS_INFO("[ROS-IGTL-Bridge] ROS-IGTL-Bridge up and Running.");
  
  this->mcpoint = new MessageConverterPoint;
  mcpoint->setup(nh, socket, 10);
  mcpoint->publish("IGTL_POINT_IN");
  mcpoint->subscribe("IGTL_POINT_OUT");

  this->mctransform = new MessageConverterTransform;
  mctransform->setup(nh, socket, 10);
  mctransform->publish("IGTL_TRANSFORM_IN");
  mctransform->subscribe("IGTL_TRANSFORM_OUT");
  
  this->mcpolydata = new MessageConverterPolyData;
  mcpolydata->setup(nh, socket, 10);
  mcpolydata->publish("IGTL_POLYDATA_IN");
  mcpolydata->subscribe("IGTL_POLYDATA_OUT");

  this->mcstring = new MessageConverterString;
  mcstring->setup(nh, socket, 10);
  mcstring->publish("IGTL_STRING_IN");
  mcstring->subscribe("IGTL_STRING_OUT");

  this->mcimage = new MessageConverterImage;
  mcimage->setup(nh, socket, 5);
  mcimage->publish("IGTL_IMAGE_IN");
  mcimage->subscribe("IGTL_IMAGE_OUT");

  this->mcpointcloud = new MessageConverterPointCloud;
  mcpointcloud->setup(nh, socket, 5);
  mcpointcloud->publish("IGTL_POINTCLOUD_IN");
  mcpointcloud->subscribe("IGTL_POINTCLOUD_OUT");

  // start receiver thread
  boost::thread* receiver_thread = new boost::thread(boost::bind(&ROS_IGTL_Bridge::IGTLReceiverThread, this));  
}

//----------------------------------------------------------------------
ROS_IGTL_Bridge::~ROS_IGTL_Bridge()
{
  socket->CloseSocket();
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
void ROS_IGTL_Bridge::CreateIGTLServer()
{
  int    port     = 18944;  // std port
  if(nh->getParam("/RIB_port",port))
    {}
  else
    {
    ROS_INFO("[ROS-IGTL-Bridge] Input socket port: ");
    std::cin >> port;
    }
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int c = serverSocket->CreateServer(port);
  
  if (c < 0)
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Cannot create a server socket.");
    }
  ROS_INFO("[ROS-IGTL-Bridge] Server socket created. Please connect to port: %d",port);
  
  // wait for connection
  while (1)
    {
    socket = serverSocket->WaitForConnection(1000);
    if (ROS_IGTL_Bridge::socket.IsNotNull()) 
      {   
      break;
      }
    }
}

//----------------------------------
void ROS_IGTL_Bridge::ConnectToIGTLServer()
{
  igtl::ClientSocket::Pointer clientsocket;
  clientsocket = igtl::ClientSocket::New();
  
  int    port     = 18944; // std port
  std::string ip;
  // get ip
  if(nh->getParam("/RIB_server_ip",ip))
    {}
  else
    {
    ROS_INFO("[ROS-IGTL-Bridge] Please enter ServerIP: ");
    std::cin >> ip;
    }
  // get port
  if(nh->getParam("/RIB_port",port))
    {}
  else
    {
    ROS_INFO("[ROS-IGTL-Bridge] Please enter ServerPort:  ");
    std::cin >> port;
    }
  // connect to server
  int r = clientsocket->ConnectToServer(ip.c_str(), port);
  
  if (r != 0)
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Cannot connect to server.");
    exit(0);
    }
  socket = (igtl::Socket *)(clientsocket);
}

// ----- receiving from slicer -----------------------------------------
//----------------------------------------------------------------------
void ROS_IGTL_Bridge::IGTLReceiverThread()
{
  igtl::MessageHeader::Pointer headerMsg;
  headerMsg = igtl::MessageHeader::New();
  int rs = 0;
  while(1)
    {
    headerMsg->InitPack();
    // receive packet
    rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
    
    if (rs == 0)
      socket->CloseSocket();
    if (rs != headerMsg->GetPackSize())
      continue;
    
    headerMsg->Unpack();
    // DATATYPE POINT ----------------------------------------------
    if (strcmp(headerMsg->GetDeviceType(), "POINT") == 0)
      {
	this->mcpoint->onIGTLMessage(headerMsg);
      }
    // DATATYPE STRING ---------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
      {
        this->mcstring->onIGTLMessage(headerMsg);
      }
    // DATATYPE TRANSFORM ------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
      { 
	this->mctransform->onIGTLMessage(headerMsg);
      }
    // DATATYPE POLYDATA -------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "POLYDATA") == 0)
      {
        this->mcpolydata->onIGTLMessage(headerMsg);
      }
    // DATATYPE IMAGE -------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "IMAGE") == 0)
      {
        this->mcimage->onIGTLMessage(headerMsg);
      }
    // SKIP DATA 
    else
      {
      socket->Skip(headerMsg->GetBodySizeToRead(),0);
      }
    }
}

