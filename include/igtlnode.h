#ifndef IGTLNODE_H
#define IGTLNODE_H
#include "ros/ros.h"
#include "std_msgs/String.h"
// IGTL Includes
#include <IGTL_Server.h>
// VTK Includes
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
// Boost Includes
#include <boost/thread.hpp>   
//----------------------------------------------------------------------
class igtlnode_class
{
public:
	igtlnode_class(int argc, char *argv[], const char* node_name);
	~igtlnode_class();
	void Run();
	
private:
	ros::NodeHandle *nh;
	ros::Publisher point_pub;
	ros::Subscriber sub_point;
	ros::Subscriber sub_posi;
	
	IGTL_Server * igtlserver;
	igtl::Socket::Pointer socket;
	
	vtkSmartPointer<vtkPoints> points_to_slicer;
	
	void CreateIGTLServer();
	void PointReceiverThread();
	void robotCallback(const std_msgs::String::ConstPtr& msg);
	void robotPositionCallback(const std_msgs::String::ConstPtr& msg);
};
#endif 
