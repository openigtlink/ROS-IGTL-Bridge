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


class MessageConverterPoint;
class MessageConverterPointCloud;
class MessageConverterTransform;
class MessageConverterPolyData;
class MessageConverterString;
class MessageConverterImage;
class MessageConverterVideo;


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
  
private:
  ros::NodeHandle *nh;

  MessageConverterPoint* mcpoint;
  MessageConverterTransform* mctransform;
  MessageConverterPolyData* mcpolydata;
  MessageConverterPointCloud* mcpointcloud;
  MessageConverterString* mcstring;
  MessageConverterImage* mcimage;
  
  // Callbacks
  virtual void IGTLReceiverThread();
  
};
#endif 
