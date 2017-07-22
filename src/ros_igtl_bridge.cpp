#include <ros_igtl_bridge.h>
#include "ShowPolyData.h"

#include "message_converter_point.h"
#include "message_converter_transform.h"
#include "message_converter_polydata.h"


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

  
  // declare publisher 
  //point_pub = nh->advertise<ros_igtl_bridge::igtlpoint>("IGTL_POINT_IN", 10);  
  //transform_pub = nh->advertise<ros_igtl_bridge::igtltransform>("IGTL_TRANSFORM_IN", 10);
  //polydata_pub = nh->advertise<ros_igtl_bridge::igtlpolydata>("IGTL_POLYDATA_IN", 1);
  string_pub = nh->advertise<ros_igtl_bridge::igtlstring>("IGTL_STRING_IN", 10);
  image_pub = nh->advertise<sensor_msgs::Image>("IGTL_IMAGE_IN", 3);
  
  // declare subscriber
  //sub_point = nh->subscribe("IGTL_POINT_OUT", 10, &ROS_IGTL_Bridge::pointCallback,this);  
  sub_pointcloud = nh->subscribe("IGTL_POINTCLOUD_OUT", 2, &ROS_IGTL_Bridge::pointcloudCallback,this);  
  //sub_transform = nh->subscribe("IGTL_TRANSFORM_OUT", 10, &ROS_IGTL_Bridge::transformCallback,this);  
  sub_string = nh->subscribe("IGTL_STRING_OUT", 20, &ROS_IGTL_Bridge::stringCallback,this); 
  sub_image = nh->subscribe("IGTL_IMAGE_OUT", 1, &ROS_IGTL_Bridge::imageCallback,this); 
  sub_video = nh->subscribe("IGTL_VIDEO_OUT", 1, &ROS_IGTL_Bridge::videoCallback,this); 
  //sub_polydata = nh->subscribe("IGTL_POLYDATA_OUT", 1, &ROS_IGTL_Bridge::polydataCallback,this); 
  
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

// ---- callbacks for sending ------------------------------------------
//----------------------------------------------------------------------
//void ROS_IGTL_Bridge::transformCallback(const ros_igtl_bridge::igtltransform::ConstPtr& msg)
//{
//  // convert msg to igtl matrix
//  igtl::Matrix4x4 Transformation;
//  igtl::IdentityMatrix(Transformation);
//  
//  // rotation
//  float quaternion [4];
//  quaternion[0] = msg->transform.rotation.x;
//  quaternion[1] = msg->transform.rotation.y;
//  quaternion[2] = msg->transform.rotation.z;
//  quaternion[3] = msg->transform.rotation.w;
//  igtl::QuaternionToMatrix(quaternion,Transformation);
//  
//  // translation
//  Transformation[0][3] = msg->transform.translation.x;
//  Transformation[1][3] = msg->transform.translation.y;
//  Transformation[2][3] = msg->transform.translation.z;
//  
//  // send transform
//  SendTransform(msg->name.c_str(),Transformation);
//}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg)
//{
//  SendPoint(msg->name.c_str(),msg->pointdata);
//}	

//----------------------------------------------------------------------	
void ROS_IGTL_Bridge::pointcloudCallback(const ros_igtl_bridge::igtlpointcloud::ConstPtr& msg)
{
  SendPointCloud(msg);
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::stringCallback(const ros_igtl_bridge::igtlstring::ConstPtr& msg)
{
  SendString(msg->name.c_str(), msg->data);
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::imageCallback(const ros_igtl_bridge::igtlimage::ConstPtr& msg)
{
  SendImage(msg);
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::videoCallback(sensor_msgs::Image::ConstPtr msg)
{
  SendVideo(msg);
}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg)
//{
//  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//  MsgToPolyData(msg,polydata);
//  SendPolyData(msg->name.c_str(),polydata);
//}

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
	//ReceivePoints(headerMsg);
	this->mcpoint->onIGTLMessage(headerMsg);
      }
    // DATATYPE STRING ---------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
      {
        ReceiveString(headerMsg);
      }
    // DATATYPE TRANSFORM ------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
      { 
	//ReceiveTransform(headerMsg);
	this->mctransform->onIGTLMessage(headerMsg);
      }
    // DATATYPE POLYDATA -------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "POLYDATA") == 0)
      {
        //vtkSmartPointer<vtkPolyData>  polydata = vtkSmartPointer<vtkPolyData>::New();
        //ReceivePolyData(headerMsg, polydata);
        this->mcpolydata->onIGTLMessage(headerMsg);
      }
    // DATATYPE IMAGE -------------------------------------------
    else if (strcmp(headerMsg->GetDeviceType(), "IMAGE") == 0){
    ReceiveImage(headerMsg);
    }
    // SKIP DATA 
    else
      {
      socket->Skip(headerMsg->GetBodySizeToRead(),0);
      }
    }
}

//// sending / receiving methods
////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::SendTransform(const char* name, igtl::Matrix4x4 &sendMatrix)
//{
//  // generate message
//  igtl::TransformMessage::Pointer transMsg;
//  transMsg = igtl::TransformMessage::New();
//  transMsg->SetDeviceName(name);
//  transMsg->SetMatrix(sendMatrix);
//  transMsg->Pack();
//  socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
//}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::ReceiveTransform(igtl::MessageHeader * header)
//{
//  // create a message buffer to receive transform data
//  igtl::TransformMessage::Pointer transMsg;
//  transMsg = igtl::TransformMessage::New();
//  transMsg->SetMessageHeader(header);
//  transMsg->AllocatePack();
//  
//  // receive transform data from the socket
//  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
//  
//  // unpack message
//  int c = transMsg->Unpack(1);
//  
//  if (c & igtl::MessageHeader::UNPACK_BODY) 
//    { 
//    // retrive the transform data
//    ros_igtl_bridge::igtltransform msg;
//    igtl::Matrix4x4 igtlmatrix;
//    igtl::IdentityMatrix(igtlmatrix);
//    transMsg->GetMatrix(igtlmatrix);
//    
//    msg.transform.translation.x = igtlmatrix[0][3];
//    msg.transform.translation.y = igtlmatrix[1][3];
//    msg.transform.translation.z = igtlmatrix[2][3];
//    
//    float quaternion [4];
//    igtl::MatrixToQuaternion(igtlmatrix,quaternion);
//    
//
//    msg.transform.rotation.x = quaternion[0];
//    msg.transform.rotation.y = quaternion[1];
//    msg.transform.rotation.z = quaternion[2];
//    msg.transform.rotation.w = quaternion[3];
//    
//    msg.name = transMsg->GetDeviceName();
//    
//    // publish to topic
//    transform_pub.publish(msg);
//    }
//  else 
//    {
//    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: TRANSFORM.");
//    return ;
//    }
//}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::SendPoint (const char* name,geometry_msgs::Point point)
//{
//  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
//  pointMsg->SetDeviceName(name);
//  
//  igtl::PointElement::Pointer pointE; 
//  pointE = igtl::PointElement::New();
//  pointE->SetPosition(point.x, point.y,point.z);
//  
//  pointMsg->AddPointElement(pointE);
//  
//  pointMsg->Pack();
//  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
//}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::SendPointCloud (const ros_igtl_bridge::igtlpointcloud::ConstPtr& msg)
{
  int pcl_size = msg->pointdata.size();
  if (!pcl_size) 
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Pointcloud is empty!");
    return;
    }
  
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());
  
  for (int i = 0; i<pcl_size;i++)
    {
    igtl::PointElement::Pointer pointE; 
    pointE = igtl::PointElement::New();
    pointE->SetPosition(msg->pointdata[i].x, msg->pointdata[i].y,msg->pointdata[i].z);		
    pointMsg->AddPointElement(pointE);
    }
  pointMsg->Pack();
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::ReceivePoints(igtl::MessageHeader * header)
//{	
//  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
//  pointMsg->SetMessageHeader(header);
//  pointMsg->AllocatePack();
//  
//  socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize());
//  int c = pointMsg->Unpack(1);
//  
//  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
//    {
//    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: POINT.");
//    return;
//    }
//  
//  int npoints = pointMsg->GetNumberOfPointElement ();
//  
//  if (npoints > 0)
//    {
//    for (int i = 0; i < npoints; i ++)
//      {
//      igtlFloat32 point[3];
//      igtl::PointElement::Pointer elem = igtl::PointElement::New();
//      pointMsg->GetPointElement (i,elem);
//      elem->GetPosition(point);
//      
//      ros_igtl_bridge::igtlpoint msg;
//      
//      msg.pointdata.x = point[0];
//      msg.pointdata.y = point[1];
//      msg.pointdata.z = point[2];
//      msg.name = elem->GetName();
//      
//      point_pub.publish(msg);
//      }
//    }
//  else
//    {
//    ROS_ERROR("[ROS-IGTL-Bridge] Message POINT is empty");
//    return;
//    }
//}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::SendImage(ros_igtl_bridge::igtlimage::ConstPtr imgmsg)
{
  int   size[]     = {imgmsg->z_steps,imgmsg->y_steps,imgmsg->x_steps};       // image dimension
  float spacing[]  = {imgmsg->z_spacing,imgmsg->y_spacing,imgmsg->x_spacing};     // spacing (mm/pixel) 
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;
  
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetCoordinateSystem(2);
  imgMsg->SetDeviceName(imgmsg->name.c_str());
  imgMsg->SetOrigin(0,0,0);
  imgMsg->AllocateScalars();
  //-----------------------------------------------------------
  
  memcpy(imgMsg->GetScalarPointer(),(char*)(&imgmsg->data[0]),imgmsg->data.size());
  
  igtl::Matrix4x4 matrixa;
  igtl::IdentityMatrix(matrixa);
  imgMsg->SetMatrix(matrixa);
  
  //------------------------------------------------------------
  // Pack and send
  imgMsg->Pack();
  
  socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::SendVideo(sensor_msgs::Image::ConstPtr imgmsg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
    cv_ptr = cv_bridge::toCvCopy(imgmsg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
  
  cv::Mat img; 
  cvtColor(cv_ptr->image,img, CV_RGB2GRAY);
  
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; 
  
  std_msgs::Header header; 
  header.stamp = ros::Time::now(); 
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
  img_bridge.toImageMsg(img_msg); 
  
  sensor_msgs::Image::Ptr msg = boost::make_shared<sensor_msgs::Image>(img_msg);
  
  /* debug window
     static const char WINDOW[] = "Image window";
     cv::namedWindow(WINDOW);
     cv::imshow(WINDOW, img);
     cv::waitKey(3);
  */
	
  int   size[]     = {imgmsg->width,imgmsg->height,1};       // image dimension
  float spacing[]  = {1,1,1};     // spacing (mm/pixel) 
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;
  
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetCoordinateSystem(2);
  imgMsg->SetDeviceName("ROS_IGTL_Bridge_Video");
  imgMsg->SetOrigin(0,0,0);
  imgMsg->AllocateScalars();
  //------------------------------------------------------------
  std::cout<< msg->data.size()<<std::endl;
  std::cout<< msg->step<<std::endl;
  std::cout<< msg->height<<std::endl;
  std::cout<< msg->width<<std::endl;
  
  image_pub.publish(msg);
  
  memcpy(imgMsg->GetScalarPointer(),(char*)(&msg->data[0]),msg->data.size());
  
  igtl::Matrix4x4 matrixa;
  igtl::IdentityMatrix(matrixa);
  matrixa[0][0] = -1;
  matrixa[1][1] = -1;
  
  imgMsg->SetMatrix(matrixa);
  
  //------------------------------------------------------------
  // Pack and send
  imgMsg->Pack();
  
  socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::ReceiveImage(igtl::MessageHeader * header)
{
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetMessageHeader(header);
  imgMsg->AllocatePack();

  socket->Receive(imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize());
  int c = imgMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    ROS_ERROR("[ROS-IGTL-Bridge] Failed to unpack the message. Datatype: IMAGE.");
    return;
    }
  
  std::vector<uint8_t> image;
  sensor_msgs::ImagePtr img_msg  (new sensor_msgs::Image()) ;
  
  float spacing[] = {0,0,0};
  imgMsg->GetSpacing(spacing);
  
  int   imgsize[] = {0,0,0};
  imgMsg->GetDimensions(imgsize);
  img_msg->height = imgsize[1];
  img_msg->width = imgsize[0];
  
  ROS_INFO("h %d",img_msg->height);
  ROS_INFO("w %d",img_msg->width);
  
  img_msg->encoding = "mono8";
  img_msg->is_bigendian = false;
  img_msg->step = imgsize[0];
  ROS_INFO("s %d",img_msg->step);
  size_t size = img_msg->step* img_msg->height;
  img_msg->data.resize(size);
  
  memcpy((char*)(&img_msg->data[0]),imgMsg->GetScalarPointer(),size); 
  
  image_pub.publish(img_msg);
}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::SendPolyData(const char* name,vtkSmartPointer<vtkPolyData> polydata)
//{
//  igtl::PolyDataMessage::Pointer polyDataMsg;
//  polyDataMsg = igtl::PolyDataMessage::New();
//  polyDataMsg->SetDeviceName(name);	
//  
//  // points
//  igtl::PolyDataPointArray::Pointer pointArray;
//  pointArray = igtl::PolyDataPointArray::New();
//  
//  double *point;
//  for (unsigned int i = 0; i <polydata->GetNumberOfPoints(); i ++)
//    {
//    point = polydata->GetPoint(i);
//    pointArray->AddPoint(static_cast<igtlFloat32>(point[0]),static_cast<igtlFloat32>(point[1]),static_cast<igtlFloat32>(point[2]));
//    }
//  polyDataMsg->SetPoints(pointArray);
//  
//  vtkSmartPointer<vtkCellArray> cellarray =
//    vtkSmartPointer<vtkCellArray>::New();
//  
//  igtl::PolyDataCellArray::Pointer polyArray;
//  polyArray = igtl::PolyDataCellArray::New();
//  
//  // polys
//  int npolys = polydata->GetNumberOfPolys();
//  if(npolys>0)
//    {
//    cellarray = polydata->GetPolys();
//    
//    vtkSmartPointer<vtkIdList> IdList =
//      vtkSmartPointer<vtkIdList>::New();
//    
//    cellarray->InitTraversal();
//    while (cellarray->GetNextCell(IdList))
//      {
//      std::list<igtlUint32> cell;
//      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
//        {
//        cell.push_back(IdList->GetId(i));
//        }
//      polyArray->AddCell(cell);
//      }
//    polyDataMsg->SetPolygons(polyArray);
//    }
//  
//  // vertices
//  int nverts = polydata->GetNumberOfVerts();
//  if(nverts>0)
//    {
//    cellarray = polydata->GetVerts();
//    
//    vtkSmartPointer<vtkIdList> IdList =
//      vtkSmartPointer<vtkIdList>::New();
//    
//    cellarray->InitTraversal();
//    while (cellarray->GetNextCell(IdList))
//      {
//      std::list<igtlUint32> cell;
//      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
//        {
//        cell.push_back(IdList->GetId(i));
//        }
//      polyArray->AddCell(cell);
//      }
//    polyDataMsg->SetVertices(polyArray);
//    }	
//
//  // lines
//  int nlines = polydata->GetNumberOfLines();
//  if(nlines>0)
//    {
//    cellarray = polydata->GetLines();
//		
//    vtkSmartPointer<vtkIdList> IdList =
//      vtkSmartPointer<vtkIdList>::New();
//		
//    cellarray->InitTraversal();
//    while (cellarray->GetNextCell(IdList))
//      {
//      std::list<igtlUint32> cell;
//      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
//        {
//        cell.push_back(IdList->GetId(i));
//        }
//      polyArray->AddCell(cell);
//      }
//    polyDataMsg->SetLines(polyArray);
//    }
//	
//  // strips
//  int nstrips = polydata->GetNumberOfStrips();
//  if(nstrips>0)
//    {
//    cellarray = polydata->GetStrips();
//		
//    vtkSmartPointer<vtkIdList> IdList =
//      vtkSmartPointer<vtkIdList>::New();
//		
//    cellarray->InitTraversal();
//    while (cellarray->GetNextCell(IdList))
//      {
//      std::list<igtlUint32> cell;
//      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
//        {
//        cell.push_back(IdList->GetId(i));
//        }
//      polyArray->AddCell(cell);
//      }
//    polyDataMsg->SetTriangleStrips(polyArray);
//    }
//  // pack and send
//  polyDataMsg->Pack();
//	
//  socket->Send(polyDataMsg->GetPackPointer(), polyDataMsg->GetPackSize());
//}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::ReceivePolyData(igtl::MessageHeader * header, vtkSmartPointer<vtkPolyData> poly)
//{
//  // Create a message buffer to receive image data
//  igtl::PolyDataMessage::Pointer polyDataMsg;
//  polyDataMsg = igtl::PolyDataMessage::New();
//  polyDataMsg->SetMessageHeader(header);
//  polyDataMsg->AllocatePack();
//	
//  socket->Receive(polyDataMsg->GetPackBodyPointer(), polyDataMsg->GetPackBodySize());
//  int c = polyDataMsg->Unpack(1);
//	
//  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) // if CRC check fails
//    {
//    std::cout<<"Unable to create PolyData from incoming POLYDATA message. Failed to unpack the message"<< std::endl;
//    return ;
//    }
//	
//  const char* name = polyDataMsg->GetDeviceName();
//	
//  // Points
//  igtl::PolyDataPointArray::Pointer pointsArray = polyDataMsg->GetPoints();
//  int npoints = pointsArray->GetNumberOfPoints();
//  if (npoints > 0)
//    {
//    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//    for (int i = 0; i < npoints; i ++)
//      {
//      igtlFloat32 point[3];
//      pointsArray->GetPoint(i, point);
//      points->InsertNextPoint(point); 
//      }
//    poly->SetPoints(points);
//    }
//	
//  // Vertices
//  igtl::PolyDataCellArray::Pointer verticesArray = polyDataMsg->GetVertices();
//  int nvertices = verticesArray->GetNumberOfCells();
//  if (nvertices > 0)
//    {
//    vtkSmartPointer<vtkCellArray> vertCells = vtkSmartPointer<vtkCellArray>::New();
//    for (int i = 0; i < nvertices; i ++)
//      {
//      vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
//			
//      std::list<igtlUint32> cell;
//      verticesArray->GetCell(i, cell);
//      std::list<igtlUint32>::iterator iter;
//      iter = cell.begin();
//      vertex->GetPointIds()->SetId(i, *iter);
//      vertCells->InsertNextCell(vertex);
//      }
//    poly->SetVerts(vertCells);
//    }
//	
//  // Lines
//  igtl::PolyDataCellArray::Pointer linesArray = polyDataMsg->GetLines();
//  int nlines = linesArray->GetNumberOfCells();
//  if (nlines > 0)
//    {
//    vtkSmartPointer<vtkCellArray> lineCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < nlines; i++)
//      {
//      vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
//			
//      std::list<igtlUint32> cell;
//      linesArray->GetCell(i, cell);
//      polyLine->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        polyLine->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      lineCells->InsertNextCell(polyLine);
//      }
//    poly->SetLines(lineCells);
//    }
//	
//  // Polygons
//  igtl::PolyDataCellArray::Pointer polygonsArray = polyDataMsg->GetPolygons();
//  int npolygons = polygonsArray->GetNumberOfCells();
//  if (npolygons > 0)
//    {
//    vtkSmartPointer<vtkCellArray> polygonCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < npolygons; i++)
//      {
//      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
//			
//      std::list<igtlUint32> cell;
//      polygonsArray->GetCell(i, cell);
//      polygon->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        polygon->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      polygonCells->InsertNextCell(polygon);
//      }
//    poly->SetPolys(polygonCells);
//    }
//	
//  // Triangle Strips
//  igtl::PolyDataCellArray::Pointer triangleStripsArray = polyDataMsg->GetTriangleStrips();
//  int ntstrips = triangleStripsArray->GetNumberOfCells();
//  if (ntstrips > 0)
//    {
//    vtkSmartPointer<vtkCellArray> tstripCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < ntstrips; i++)
//      {
//      vtkSmartPointer<vtkTriangleStrip> tstrip = vtkSmartPointer<vtkTriangleStrip>::New();
//			
//      std::list<igtlUint32> cell;
//      triangleStripsArray->GetCell(i, cell);
//      tstrip->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        tstrip->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      tstripCells->InsertNextCell(tstrip);
//      }
//    poly->SetStrips(tstripCells);
//    }
//	
//  poly->Modified();
//  //Show_Polydata(poly);
//  polydata_pub.publish(PolyDataToMsg(name,poly));
//
//}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::SendString(const char* name, std::string stringmsg)
{
  igtl::StringMessage::Pointer stringMsg = igtl::StringMessage::New();
  stringMsg->SetDeviceName(name);
  stringMsg->SetString(stringmsg.c_str());  
  stringMsg->Pack();
  //std::cout<<stringmsg.c_str()<< " sent"<<std::endl;
  socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
}

//----------------------------------------------------------------------
void ROS_IGTL_Bridge::ReceiveString(igtl::MessageHeader * header)
{
  // Create a message buffer to receive string data
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();

  // Receive string data from the socket
  socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());

  int b = stringMsg->Unpack(1);
  ros_igtl_bridge::igtlstring msg;
	
  if (b & igtl::MessageHeader::UNPACK_BODY) 
    {
    //std::cout<< "Received String: "<<stringMsg->GetString()<<std::endl;     
    msg.name = stringMsg->GetDeviceName();
    msg.data = stringMsg->GetString();
    string_pub.publish(msg);
    }
  else 
    {
    std::cerr << "CRC Check failed!" << std::endl;
    return ;
    }
}

//// Polydata/Msg conversions
////----------------------------------------------------------------------
//ros_igtl_bridge::igtlpolydata ROS_IGTL_Bridge::PolyDataToMsg(const char* name, vtkSmartPointer<vtkPolyData> polydata )
//{
//  ros_igtl_bridge::igtlpolydata msg;
//
//  msg.name=name;
//	
//  // points
//  double *point;
//  msg.points.resize(polydata->GetNumberOfPoints());
//	
//  for (unsigned int i = 0; i <polydata->GetNumberOfPoints(); i ++)
//    {
//    point = polydata->GetPoint(i);
//    msg.points[i].x-=point[0];
//    msg.points[i].y-=point[1];
//    msg.points[i].z =point[2];
//    }
//	
//  // vertices
//  int nverts = polydata->GetNumberOfVerts();
//  if(nverts>0)
//    {
//    msg.verts.resize(nverts);
//		
//    for (unsigned int i = 0; i<nverts;i++)
//      {
//      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
//      polydata->GetCellPoints(i,IdList);
//      std::vector<igtlUint32> vec;
//      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
//        {	
//        vec.push_back(IdList->GetId(j));
//        }
//      for (int k = 0;k < vec.size();k++)
//        msg.verts[i].data.push_back(vec[k]);
//
//      }
//    }
//
//  // lines
//  int nlines = polydata->GetNumberOfLines();
//  if(nlines>0)
//    {
//    msg.lines.resize(nlines);
//		
//    for (unsigned int i = 0; i<nlines;i++)
//      {
//      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
//      polydata->GetCellPoints(i,IdList);
//      std::vector<igtlUint32> vec;
//      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
//        {	
//        vec.push_back(IdList->GetId(j));
//        }
//      for (int k = 0;k < vec.size();k++)
//        msg.lines[i].data.push_back(vec[k]);
//
//      }
//    }
//	
//  // strips
//  int ntstrips = polydata->GetNumberOfStrips();
//  if(ntstrips>0)
//    {
//    msg.strips.resize(ntstrips);
//		
//    for (unsigned int i = 0; i<ntstrips;i++)
//      {
//      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
//      polydata->GetCellPoints(i,IdList);
//      std::vector<igtlUint32> vec;
//      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
//        {	
//        vec.push_back(IdList->GetId(j));
//        }
//      for (int k = 0;k < vec.size();k++)
//        msg.strips[i].data.push_back(vec[k]);
//
//      }
//    }
//	
//  // polygons
//  int pnumber = pnumber = polydata->GetNumberOfPolys();
//  if(pnumber>0)
//    {
//    msg.polygons.resize(pnumber);
//
//    for (unsigned int i = 0; i<pnumber;i++)
//      {
//      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
//      polydata->GetCellPoints(i,IdList);
//      std::vector<igtlUint32> vec;
//      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
//        {	
//        vec.push_back(IdList->GetId(j));
//        }
//      msg.polygons[i].x = vec[0];
//      msg.polygons[i].y = vec[1];
//      msg.polygons[i].z = vec[2];
//      }
//    }
//
//  return msg;
//}

////----------------------------------------------------------------------
//void ROS_IGTL_Bridge::MsgToPolyData( const ros_igtl_bridge::igtlpolydata::ConstPtr& msg, vtkSmartPointer<vtkPolyData> polydata)
//{
//  // points
//  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//  for (int i = 0; i < msg->points.size(); i ++)
//    {
//    double point[3];
//    point[0] = msg->points[i].x;
//    point[1] = msg->points[i].y;
//    point[2] = msg->points[i].z;
//    points->InsertNextPoint(point); 
//    }
//  polydata->SetPoints(points);
//
//  // strips
//  int ntstrips = msg->strips.size();
//  if (ntstrips > 0)
//    {
//    vtkSmartPointer<vtkCellArray> tstripCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < ntstrips; i++)
//      {
//      vtkSmartPointer<vtkTriangleStrip> tstrip = vtkSmartPointer<vtkTriangleStrip>::New();
//			
//      std::list<igtlUint32> cell;
//      for (int k = 0;k < msg->strips[i].data.size();k++)
//        cell.push_back(msg->strips[i].data[k]);
//			
//      tstrip->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        tstrip->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      tstripCells->InsertNextCell(tstrip);
//      }
//    polydata->SetStrips(tstripCells);
//    }
//	
//  // lines
//  int nlines = msg->lines.size();
//  if (nlines > 0)
//    {
//    vtkSmartPointer<vtkCellArray> linesCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < nlines; i++)
//      {
//      vtkSmartPointer<vtkPolyLine> lines = vtkSmartPointer<vtkPolyLine>::New();
//      std::list<igtlUint32> cell;
//      for (int k = 0;k < msg->lines[i].data.size();k++)
//        cell.push_back(msg->lines[i].data[k]);
//		
//      lines->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        lines->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      linesCells->InsertNextCell(lines);
//      }
//    polydata->SetLines(linesCells);
//    }
//	
//  // verts
//  int nverts = msg->verts.size();
//  if (nverts > 0)
//    {
//    vtkSmartPointer<vtkCellArray> vertCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < nlines; i++)
//      {
//      vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
//      std::list<igtlUint32> cell;
//      for (int k = 0;k < msg->verts[i].data.size();k++)
//        cell.push_back(msg->verts[i].data[k]);
//		
//      vertex->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        vertex->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      vertCells->InsertNextCell(vertex);
//      }
//    polydata->SetVerts(vertCells);
//    }
//
//  // Polygons
//  int npolygons = msg->polygons.size();
//  if (npolygons > 0)
//    {
//    vtkSmartPointer<vtkCellArray> polygonCells = vtkSmartPointer<vtkCellArray>::New();
//    for(int i = 0; i < npolygons; i++)
//      {
//      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();		
//      std::list<igtlUint32> cell;
//      cell.push_back(msg->polygons[i].x);
//      cell.push_back(msg->polygons[i].y);
//      cell.push_back(msg->polygons[i].z);
//      polygon->GetPointIds()->SetNumberOfIds(cell.size());
//      std::list<igtlUint32>::iterator iter;
//      int j = 0;
//      for (iter = cell.begin(); iter != cell.end(); iter ++)
//        {
//        polygon->GetPointIds()->SetId(j, *iter);
//        j++;
//        }
//      polygonCells->InsertNextCell(polygon);
//			
//      }
//    polydata->SetPolys(polygonCells);
//    }
//  polydata->Modified();
//}
