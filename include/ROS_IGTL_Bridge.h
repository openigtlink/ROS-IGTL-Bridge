#ifndef ROS_IGTL_BRIDGE_H
#define ROS_IGTL_BRIDGE_H
// ROS Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Transform.h"
// OpenCV Includes
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Boost Includes
#include <boost/thread.hpp>   
// Messages Includes
#include <ros_igtl_bridge/igtltransform.h>
#include <ros_igtl_bridge/igtlpoint.h>
#include <ros_igtl_bridge/igtlpointcloud.h>
#include <ros_igtl_bridge/igtlpolydata.h>
#include <ros_igtl_bridge/igtlimage.h>
#include <ros_igtl_bridge/igtlstring.h>
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
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkVertex.h>
#include <vtkPolyLine.h>
#include <vtkTriangleStrip.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
// C++ Includes
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>

class ROS_IGTL_Bridge
{
public:
	ROS_IGTL_Bridge(int argc, char *argv[], const char* node_name);
	~ROS_IGTL_Bridge();
	void Run();

	igtl::Socket::Pointer socket;
	igtl::Socket::Pointer GetSocketPointer();
	virtual void CreateIGTLServer();
	virtual void ConnectToIGTLServer();
	
	// PD Message Conversions
	static ros_igtl_bridge::igtlpolydata PolyDataToMsg(const char* name,vtkSmartPointer<vtkPolyData> polydata);
	static void MsgToPolyData(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg, vtkSmartPointer<vtkPolyData> polydata);
	
private:
	// Publisher/Subscribers 
	ros::NodeHandle *nh;
	ros::Publisher point_pub;
	ros::Publisher transform_pub;
	ros::Publisher polydata_pub;
	ros::Publisher image_pub;
	ros::Publisher string_pub;
	ros::Subscriber sub_point;
	ros::Subscriber sub_pointcloud;
	ros::Subscriber sub_transform;
	ros::Subscriber sub_polydata;
	ros::Subscriber sub_image;
	ros::Subscriber sub_video;
	ros::Subscriber sub_string;
	
	// Callbacks
	virtual void IGTLReceiverThread();
	virtual void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg);
	virtual void pointcloudCallback(const ros_igtl_bridge::igtlpointcloud::ConstPtr& msg);
	virtual void transformCallback(const ros_igtl_bridge::igtltransform::ConstPtr& msg);
	virtual void stringCallback(const ros_igtl_bridge::igtlstring::ConstPtr& msg);
	virtual void imageCallback(const ros_igtl_bridge::igtlimage::ConstPtr& msg);
	virtual void videoCallback(sensor_msgs::Image::ConstPtr msg);
	virtual void polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg);

	// Sending
	virtual void SendTransform(const char* name, igtl::Matrix4x4 &sendMatrix);
	virtual void SendPoint (const char* name,geometry_msgs::Point point);
	virtual void SendPointCloud (const ros_igtl_bridge::igtlpointcloud::ConstPtr& msg);
	virtual void SendImage(ros_igtl_bridge::igtlimage::ConstPtr imgmsg);
	virtual void SendVideo(sensor_msgs::Image::ConstPtr imgmsg);
	virtual void SendPolyData(const char* name,vtkSmartPointer<vtkPolyData> polydata);
	virtual void SendString(const char* name, std::string stringmsg);

	// Receiving
	virtual void ReceivePoints(igtl::MessageHeader * header);
	virtual void ReceiveImage(igtl::MessageHeader * header);
	virtual void ReceiveTransform(igtl::MessageHeader * header);
	virtual void ReceivePolyData(igtl::MessageHeader * header,vtkSmartPointer<vtkPolyData> poly);
	virtual void ReceiveString(igtl::MessageHeader * header);	
};
#endif 
