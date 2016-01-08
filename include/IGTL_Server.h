#ifndef IGTL_SERVER_H
#define IGTL_SERVER_H
// ROS Includes
#include "ros/ros.h"
// IGTL Includes
#include "igtlImageMessage.h"
#include "igtlMessageHeader.h"
#include "igtlMessageBase.h"
#include "igtlMath.h"
#include "igtlOSUtil.h"
#include "igtlPolyDataMessage.h"
#include "igtlServerSocket.h"
#include "igtlSmartPointer.h"
#include "igtlStatusMessage.h"
#include "igtlStringMessage.h"
#include "igtlTransformMessage.h"
#include "igtlPointMessage.h"
// VTK Includes
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
// ---------------------------------------------------------------------
class IGTL_Server
{
public:
    IGTL_Server(igtl::Socket::Pointer m_socket);
     ~IGTL_Server();
	
	// Sending
	void SendTransform(const char* name, igtl::Matrix4x4 &sendMatrix);
	void SendPoints (const char* name,vtkSmartPointer<vtkPoints> points);

	//int SendPolyData(const char* name,vtkSmartPointer<vtkPolyData> polydata);
	//void SendImage(const char* name, std::vector<uint8_t> image, octScanParameter_t scanparameter, igtl::Matrix4x4 *matrix);
	//void SendAck(std::string AckString);

	// Receiving
	const char* ReceiveFromSlicer(vtkSmartPointer<vtkPoints> points, std::string datatype);
	void ReceivePoints(igtl::Socket * socket, igtl::MessageHeader * header, vtkSmartPointer<vtkPoints> points);

	//void ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header,igtl::Matrix4x4 *matrix);
	//void ReceivePolyData(igtl::Socket * socket, igtl::MessageHeader * header,vtkSmartPointer<vtkPolyData> poly);
	//const char* ReceiveString(igtl::Socket * socket, igtl::MessageHeader * header);
  
private:
	igtl::Socket::Pointer socket;
};
#endif



