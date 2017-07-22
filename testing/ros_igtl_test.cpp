#include <ros_igtl_test.h>
#include "ShowPolyData.h"
#include "ReadOCTFile.h"



//----------------------------------------------------------------------
ROS_IGTL_Test::ROS_IGTL_Test(int argc, char *argv[], const char* node_name)
{
  ros::init(argc, argv, node_name);
  nh = new ros::NodeHandle;	
  ROS_INFO("[Test-Node] Test-Node up and Running.");	
  	
  // declare publisher 
  point_pub = nh->advertise<ros_igtl_bridge::igtlpoint>("IGTL_POINT_OUT", 10);  
  transform_pub = nh->advertise<ros_igtl_bridge::igtltransform>("IGTL_TRANSFORM_OUT", 10);
  polydata_pub = nh->advertise<ros_igtl_bridge::igtlpolydata>("IGTL_POLYDATA_OUT", 1);
  string_pub = nh->advertise<ros_igtl_bridge::igtlstring>("IGTL_STRING_OUT", 15);
  image_pub = nh->advertise<ros_igtl_bridge::igtlimage>("IGTL_IMAGE_OUT", 3);
  pointcloud_pub = nh->advertise<ros_igtl_bridge::igtlpointcloud>("IGTL_POINTCLOUD_OUT", 2);  
  // declare subscriber
  sub_point = nh->subscribe("IGTL_POINT_IN", 10, &ROS_IGTL_Test::pointCallback,this);  
  sub_transform = nh->subscribe("IGTL_TRANSFORM_IN", 10, &ROS_IGTL_Test::transformCallback,this);  
  sub_string = nh->subscribe("IGTL_STRING_IN", 20, &ROS_IGTL_Test::stringCallback,this); 
  sub_image = nh->subscribe("IGTL_IMAGE_IN", 1, &ROS_IGTL_Test::imageCallback,this); 
  sub_polydata = nh->subscribe("IGTL_POLYDATA_IN", 1, &ROS_IGTL_Test::polydataCallback,this); 

  test_sending();
	
  ROS_INFO("[Test-Node] Sending Test finished!");
}

//----------------------------------------------------------------------
ROS_IGTL_Test::~ROS_IGTL_Test()
{
	 
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::Run()
{
  ros::spin();
}

//---callbackcs for subscriber------------------------------------------
//----------------------------------------------------------------------
void ROS_IGTL_Test::transformCallback(const ros_igtl_bridge::igtltransform::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] Transform %s received: \n",msg->name.c_str());
	
  // convert msg to igtl matrix
  igtl::Matrix4x4 igtlmatrix;
  igtl::IdentityMatrix(igtlmatrix);	

  float quaternion [4];
  igtl::MatrixToQuaternion(igtlmatrix,quaternion);

  quaternion[0] = msg->transform.rotation.x;
  quaternion[1] = msg->transform.rotation.y;
  quaternion[2] = msg->transform.rotation.z;
  quaternion[3] = msg->transform.rotation.w;
	
  igtl::QuaternionToMatrix(quaternion,igtlmatrix);

  igtlmatrix[0][3] = msg->transform.translation.x;
  igtlmatrix[1][3] = msg->transform.translation.y;
  igtlmatrix[2][3] = msg->transform.translation.z;
			
  igtl::PrintMatrix(igtlmatrix);
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] Point %s received: \n",msg->name.c_str());
  ROS_INFO("x = %f  y = %f  z = %f \n",msg->pointdata.x,msg->pointdata.y,msg->pointdata.z);
}
	
//----------------------------------------------------------------------
void ROS_IGTL_Test::stringCallback(const ros_igtl_bridge::igtlstring::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] String %s received: \n",msg->name.c_str());
  ROS_INFO("%s \n",msg->data.c_str());
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] Image received: \n");
  ROS_INFO("Topic: /IGTL_IMAGE_IN  Use rviz for visualization!\n");
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] PolyData %s received: \n",msg->name.c_str());
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  // TODO: Remove the dependency on ROS_IGTL_Bridge::msgToPolyData();
  //ROS_IGTL_Bridge::msgToPolyData(msg,polydata);
  std::cout<<"Number of Points "<<polydata->GetNumberOfPoints()<<std::endl;
  std::cout<<"Number of Strips "<<polydata->GetNumberOfStrips()<<std::endl;
  Show_Polydata(polydata);
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::test_sending()
{
  srand((unsigned int)time(NULL));
	
  // wait for subscribers
  while(!transform_pub.getNumSubscribers())
    {}
  while(!string_pub.getNumSubscribers())
    {}
  while(!point_pub.getNumSubscribers())
    {}
  while(!pointcloud_pub.getNumSubscribers())
    {}
  while(!polydata_pub.getNumSubscribers())
    {}
	
  //-----------------------------------
  // send transform
  ros_igtl_bridge::igtltransform transform_msg;
  igtl::Matrix4x4 igtlmatrix;
  igtl::IdentityMatrix(igtlmatrix);
	
  igtlmatrix[0][3] = (float)rand()/(float)(RAND_MAX) * 10;
  igtlmatrix[1][3] = (float)rand()/(float)(RAND_MAX) * 10;
  igtlmatrix[2][3] = (float)rand()/(float)(RAND_MAX) * 10;

  float quaternion [4];
  igtl::MatrixToQuaternion(igtlmatrix,quaternion);

  transform_msg.transform.translation.x = igtlmatrix[0][3];
  transform_msg.transform.translation.y = igtlmatrix[1][3];
  transform_msg.transform.translation.z = igtlmatrix[2][3];
  transform_msg.transform.rotation.x = quaternion[0];
  transform_msg.transform.rotation.y = quaternion[1];
  transform_msg.transform.rotation.z = quaternion[2];
  transform_msg.transform.rotation.w = quaternion[3];
	
  transform_msg.name = "ROS_IGTL_Test_Transform";
	
  // publish to topic
  transform_pub.publish(transform_msg);
	
  //-----------------------------------
  // send points


  float max = 50.0;
  std::string point_name = "ROS_IGTL_Test_Point";
  ros_igtl_bridge::igtlpoint point_msg;
	
  point_msg.pointdata.x = (float)rand()/(float)(RAND_MAX) * max;
  point_msg.pointdata.y = (float)rand()/(float)(RAND_MAX) * max;
  point_msg.pointdata.z = (float)rand()/(float)(RAND_MAX) * max;
  point_msg.name = point_name;

  point_pub.publish(point_msg);
	
  //-----------------------------------
  // send pointcloud

  int npoints = 20;
  std::string points_name = "ROS_IGTL_Test_Pointcloud";
  ros_igtl_bridge::igtlpointcloud points_msg;
  points_msg.pointdata.resize(npoints);
  for (int i = 0; i < npoints; i ++)
    {
      points_msg.pointdata[i].x = (float)rand()/(float)(RAND_MAX) * max;
      points_msg.pointdata[i].y = (float)rand()/(float)(RAND_MAX) * max;
      points_msg.pointdata[i].z = (float)rand()/(float)(RAND_MAX) * max;
		
    }
  points_msg.name = points_name;	
  pointcloud_pub.publish(points_msg);
	
  // -----------------------------------------------------------------
  // send string
	
  ros_igtl_bridge::igtlstring string_msg;
  string_msg.name = "ROS_IGTL_Test_String";
  string_msg.data = "Test1";
  string_pub.publish(string_msg);
	
  // -----------------------------------------------------------------
  // send PD
  std::string test_pd;
  if(nh->getParam("/test_pd",test_pd))
    {
      vtkSmartPointer<vtkPolyData> polydata;
      vtkSmartPointer<vtkPolyDataReader> polydata_reader = vtkSmartPointer<vtkPolyDataReader>::New();
      polydata_reader->SetFileName(test_pd.c_str());
      polydata_reader->Update();
      polydata = polydata_reader->GetOutput();

      // TODO: Remove the dependency on ROS_IGTL_Bridge::PolyDataToMs()
      //ros_igtl_bridge::igtlpolydata::Ptr polydata_msg (new ros_igtl_bridge::igtlpolydata());
      //*polydata_msg =  ROS_IGTL_Bridge::PolyDataToMsg("ROS_IGTL_Test_PolyData",polydata);
      //polydata_pub.publish(*polydata_msg);
    }
  std::string test_oct;
  if(nh->getParam("/test_oct",test_oct))
    {
      //	PublishOCTScan(test_oct);
    }
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::PublishOCTScan(std::string filepath)
{
  std::vector<uint8_t> OCTRawData;
  readVector(filepath.c_str(), OCTRawData);
	
  octScanParameter_t scanParameter;
	
  scanParameter.x_steps = 512;
  scanParameter.y_steps = 512;
  scanParameter.z_steps = 512;
  scanParameter.x_range = 10;
  scanParameter.y_range = 10;
  scanParameter.z_range = 3;
  scanParameter.x_spacing = scanParameter.x_range/scanParameter.x_steps;
  scanParameter.y_spacing = scanParameter.y_range/scanParameter.y_steps;
  scanParameter.z_spacing = scanParameter.z_range/scanParameter.z_steps;
	
  ros_igtl_bridge::igtlimage img_msg ;
				
  img_msg.x_steps = scanParameter.x_steps;
  img_msg.y_steps = scanParameter.y_steps;
  img_msg.z_steps = scanParameter.z_steps;
	
  img_msg.x_spacing = scanParameter.x_spacing;
  img_msg.y_spacing = scanParameter.y_spacing;
  img_msg.z_spacing = scanParameter.z_spacing;

  size_t size = img_msg.x_steps*img_msg.y_steps*img_msg.z_steps;
  img_msg.data.resize(size);
  img_msg.name = "ROS_IGTL_Test_Image";
  memcpy((char*)(&img_msg.data[0]),OCTRawData.data(),size); 

  image_pub.publish(img_msg);
}
