#include <IGTL_Server.h>
//----------------------------------------------------------------------
IGTL_Server::IGTL_Server(igtl::Socket::Pointer m_socket) : socket(m_socket)
{
	int    port     = 18944;
	ROS_INFO("[IGTL_Server] Input socket port: ");
	std::cin >> port;
	igtl::ServerSocket::Pointer serverSocket;
	serverSocket = igtl::ServerSocket::New();
	int c = serverSocket->CreateServer(port);

	if (c < 0)
	{
		ROS_ERROR("[IGTL_Server] Cannot create a server socket.");
	}
	ROS_INFO("[IGTL_Server] Server socket created. Please connect to port: %d",port);
	
	while (1)
	{
		socket = serverSocket->WaitForConnection(1000);
		if (socket.IsNotNull()) 
		{   
			break;
		}
	}
}
//----------------------------------------------------------------------
IGTL_Server::~IGTL_Server()
{
    socket->CloseSocket();
}
//----------------------------------------------------------------------
const char* IGTL_Server::ReceiveFromSlicer(vtkSmartPointer<vtkPoints> points, std::string datatype)
{
	igtl::MessageHeader::Pointer headerMsg;
	headerMsg = igtl::MessageHeader::New();
	
	ROS_INFO("[IGTL_Server] Waiting for Data from Slicer!");
	while(1)
	{
		headerMsg->InitPack();

		int rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
		if (rs == 0)
				socket->CloseSocket();
		if (rs != headerMsg->GetPackSize())
			continue;
		
		headerMsg->Unpack();
		
		
		// DATATYPE POINT ----------------------------------------------
		if (strcmp(headerMsg->GetDeviceType(), "POINT") == 0 && strcmp(datatype.c_str(), "POINT")== 0){
			 ReceivePoints(socket, headerMsg, points);
			 ROS_INFO("[IGTL_Server] Received POINT from Slicer");
			 return "POINT";
		}
		// -------------------------------------------------------------
		// DATATYPE ...
		//else if(...)
		//{
		//	...	
		//}
		// -------------------------------------------------------------
		else
		{
			socket->Skip(headerMsg->GetBodySizeToRead(),0);
		}
	}
}
//----------------------------------------------------------------------
void IGTL_Server::SendTransform(const char* name, igtl::Matrix4x4 &sendMatrix)
{
	igtl::TransformMessage::Pointer transMsg;
	transMsg = igtl::TransformMessage::New();
	transMsg->SetDeviceName(name);

	transMsg->SetMatrix(sendMatrix);
	transMsg->Pack();
	socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
}
//----------------------------------------------------------------------
void IGTL_Server::ReceivePoints(igtl::Socket * socket, igtl::MessageHeader * header, vtkSmartPointer<vtkPoints> points)
{	
	igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
	pointMsg->SetMessageHeader(header);
	pointMsg->AllocatePack();
	
	socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize());
	int c = pointMsg->Unpack(1);
	
	if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
	{
		ROS_ERROR("[IGTL_Server] Failed to unpack the message. Datatype: POINT.");
		return;
	}

	int npoints = pointMsg->GetNumberOfPointElement ();

	if (npoints > 0)
	{
		for (int i = 0; i < npoints; i ++)
		{
			igtlFloat32 point[3];
			igtl::PointElement::Pointer elem = igtl::PointElement::New();
			pointMsg->GetPointElement (i,elem);
			elem->GetPosition(point);
			points->InsertNextPoint(point); 
		}
	}
	else
	{
		ROS_ERROR("[IGTL_Server] Message POINT is empty");
		return;
	}
	ROS_INFO("[IGTL_Server] [%d] points received.", npoints);
	points->Modified();
}
//----------------------------------------------------------------------
void IGTL_Server::SendPoints (const char* name,vtkSmartPointer<vtkPoints> points)
{
 	igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
 	pointMsg->SetDeviceName(name);
 	int numPoints = 0;
 	numPoints = points->GetNumberOfPoints ();
 	
 	for (int i = 0; i<numPoints; i++)
 	{
		igtl::PointElement::Pointer point;
		point = igtl::PointElement::New();
		double f_point [3];
		points->GetPoint (i,f_point);
		point->SetPosition(f_point[0], f_point[1],f_point[2]);
		
		pointMsg->AddPointElement(point);
	}
	pointMsg->Pack();
	socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
	ROS_INFO("[IGTL_Server] Message POINT sent.");
}
//----------------------------------------------------------------------
//void IGTL_Server::SendImage(const char* name, std::vector<uint8_t> image, octScanParameter_t scanparameter, igtl::Matrix4x4 *matrix)
//{
// 	....	
//}
//----------------------------------------------------------------------
//int IGTL_Server::SendPolyData( const char* name,vtkSmartPointer<vtkPolyData> polydata)
//{
//	...
//}
//----------------------------------------------------------------------
//const char* IGTL_Server::ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header)
//{
// 	...
//}
//----------------------------------------------------------------------
//void IGTL_Server::ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header, igtl::Matrix4x4 *matrix)
//{
//	...
//}
//----------------------------------------------------------------------
//void IGTL_Server::ReceivePolyData(igtl::Socket * socket, igtl::MessageHeader * header, vtkSmartPointer<vtkPolyData> poly)
//{
//	...
//}
//----------------------------------------------------------------------
//void IGTL_Server::SendAck(std::string AckString)
//{
//	...
//}
//----------------------------------------------------------------------
