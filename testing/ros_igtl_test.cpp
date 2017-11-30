/*=========================================================================

  Program:   ROS-IGTL-Bridge Test Server
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "ros_igtl_test.h"
#include "ShowPolyData.h"
#include "ReadOCTFile.h"
#include "igtlMath.h"

// VTK Includes
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkCubeSource.h>
#include <vtkVertex.h>
#include <vtkPolyLine.h>
#include <vtkTriangleStrip.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkPolyDataReader.h>

#include <igtlTypes.h>

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
//void ROS_IGTL_Test::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
void ROS_IGTL_Test::imageCallback(const ros_igtl_bridge::igtlimage::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] Image received: \n");
  //ROS_INFO("Topic: /IGTL_IMAGE_IN  Use rviz for visualization!\n");
}

//----------------------------------------------------------------------
void ROS_IGTL_Test::polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg)
{
  ROS_INFO("[ROS_IGTL_Test] PolyData %s received: \n",msg->name.c_str());
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  msgToPolyData(msg,polydata);
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
      ros_igtl_bridge::igtlpolydata::Ptr polydata_msg (new ros_igtl_bridge::igtlpolydata());
      *polydata_msg =  polyDataToMsg("ROS_IGTL_Test_PolyData",polydata);
      polydata_pub.publish(*polydata_msg);
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


ros_igtl_bridge::igtlpolydata ROS_IGTL_Test::polyDataToMsg(const char* name, vtkSmartPointer<vtkPolyData> polydata )
{
  ros_igtl_bridge::igtlpolydata msg;

  msg.name=name;
	
  // points
  double *point;
  msg.points.resize(polydata->GetNumberOfPoints());
	
  for (unsigned int i = 0; i <polydata->GetNumberOfPoints(); i ++)
    {
    point = polydata->GetPoint(i);
    msg.points[i].x-=point[0];
    msg.points[i].y-=point[1];
    msg.points[i].z =point[2];
    }
	
  // vertices
  int nverts = polydata->GetNumberOfVerts();
  if(nverts>0)
    {
    msg.verts.resize(nverts);
		
    for (unsigned int i = 0; i<nverts;i++)
      {
      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
      polydata->GetCellPoints(i,IdList);
      std::vector<igtlUint32> vec;
      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
        {	
        vec.push_back(IdList->GetId(j));
        }
      for (int k = 0;k < vec.size();k++)
        msg.verts[i].data.push_back(vec[k]);

      }
    }

  // lines
  int nlines = polydata->GetNumberOfLines();
  if(nlines>0)
    {
    msg.lines.resize(nlines);
		
    for (unsigned int i = 0; i<nlines;i++)
      {
      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
      polydata->GetCellPoints(i,IdList);
      std::vector<igtlUint32> vec;
      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
        {	
        vec.push_back(IdList->GetId(j));
        }
      for (int k = 0;k < vec.size();k++)
        msg.lines[i].data.push_back(vec[k]);

      }
    }
	
  // strips
  int ntstrips = polydata->GetNumberOfStrips();
  if(ntstrips>0)
    {
    msg.strips.resize(ntstrips);
		
    for (unsigned int i = 0; i<ntstrips;i++)
      {
      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
      polydata->GetCellPoints(i,IdList);
      std::vector<igtlUint32> vec;
      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
        {	
        vec.push_back(IdList->GetId(j));
        }
      for (int k = 0;k < vec.size();k++)
        msg.strips[i].data.push_back(vec[k]);

      }
    }
	
  // polygons
  int pnumber = pnumber = polydata->GetNumberOfPolys();
  if(pnumber>0)
    {
    msg.polygons.resize(pnumber);

    for (unsigned int i = 0; i<pnumber;i++)
      {
      vtkSmartPointer<vtkIdList> IdList = vtkSmartPointer<vtkIdList>::New();
      polydata->GetCellPoints(i,IdList);
      std::vector<igtlUint32> vec;
      for (int j = 0; j < IdList->GetNumberOfIds(); j++)
        {	
        vec.push_back(IdList->GetId(j));
        }
      msg.polygons[i].x = vec[0];
      msg.polygons[i].y = vec[1];
      msg.polygons[i].z = vec[2];
      }
    }

  return msg;
}


//----------------------------------------------------------------------
void ROS_IGTL_Test::msgToPolyData(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg, vtkSmartPointer<vtkPolyData> polydata)
{
  // points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (int i = 0; i < msg->points.size(); i ++)
    {
    double point[3];
    point[0] = msg->points[i].x;
    point[1] = msg->points[i].y;
    point[2] = msg->points[i].z;
    points->InsertNextPoint(point); 
    }
  polydata->SetPoints(points);

  // strips
  int ntstrips = msg->strips.size();
  if (ntstrips > 0)
    {
    vtkSmartPointer<vtkCellArray> tstripCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < ntstrips; i++)
      {
      vtkSmartPointer<vtkTriangleStrip> tstrip = vtkSmartPointer<vtkTriangleStrip>::New();
			
      std::list<igtlUint32> cell;
      for (int k = 0;k < msg->strips[i].data.size();k++)
        cell.push_back(msg->strips[i].data[k]);
			
      tstrip->GetPointIds()->SetNumberOfIds(cell.size());
      std::list<igtlUint32>::iterator iter;
      int j = 0;
      for (iter = cell.begin(); iter != cell.end(); iter ++)
        {
        tstrip->GetPointIds()->SetId(j, *iter);
        j++;
        }
      tstripCells->InsertNextCell(tstrip);
      }
    polydata->SetStrips(tstripCells);
    }
	
  // lines
  int nlines = msg->lines.size();
  if (nlines > 0)
    {
    vtkSmartPointer<vtkCellArray> linesCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < nlines; i++)
      {
      vtkSmartPointer<vtkPolyLine> lines = vtkSmartPointer<vtkPolyLine>::New();
      std::list<igtlUint32> cell;
      for (int k = 0;k < msg->lines[i].data.size();k++)
        cell.push_back(msg->lines[i].data[k]);
		
      lines->GetPointIds()->SetNumberOfIds(cell.size());
      std::list<igtlUint32>::iterator iter;
      int j = 0;
      for (iter = cell.begin(); iter != cell.end(); iter ++)
        {
        lines->GetPointIds()->SetId(j, *iter);
        j++;
        }
      linesCells->InsertNextCell(lines);
      }
    polydata->SetLines(linesCells);
    }
	
  // verts
  int nverts = msg->verts.size();
  if (nverts > 0)
    {
    vtkSmartPointer<vtkCellArray> vertCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < nlines; i++)
      {
      vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
      std::list<igtlUint32> cell;
      for (int k = 0;k < msg->verts[i].data.size();k++)
        cell.push_back(msg->verts[i].data[k]);
		
      vertex->GetPointIds()->SetNumberOfIds(cell.size());
      std::list<igtlUint32>::iterator iter;
      int j = 0;
      for (iter = cell.begin(); iter != cell.end(); iter ++)
        {
        vertex->GetPointIds()->SetId(j, *iter);
        j++;
        }
      vertCells->InsertNextCell(vertex);
      }
    polydata->SetVerts(vertCells);
    }

  // Polygons
  int npolygons = msg->polygons.size();
  if (npolygons > 0)
    {
    vtkSmartPointer<vtkCellArray> polygonCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < npolygons; i++)
      {
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();		
      std::list<igtlUint32> cell;
      cell.push_back(msg->polygons[i].x);
      cell.push_back(msg->polygons[i].y);
      cell.push_back(msg->polygons[i].z);
      polygon->GetPointIds()->SetNumberOfIds(cell.size());
      std::list<igtlUint32>::iterator iter;
      int j = 0;
      for (iter = cell.begin(); iter != cell.end(); iter ++)
        {
        polygon->GetPointIds()->SetId(j, *iter);
        j++;
        }
      polygonCells->InsertNextCell(polygon);
			
      }
    polydata->SetPolys(polygonCells);
    }
  polydata->Modified();
}

