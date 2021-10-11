/*=========================================================================

  Program:   Converter Class for Polydata
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_polydata.h"
#include "rib_converter_manager.h"
#include "ros/ros.h"
#include "igtlPolyDataMessage.h"

// VTK header files
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkVertex.h>
#include <vtkPolyLine.h>
#include <vtkTriangleStrip.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>

RIBConverterPolyData::RIBConverterPolyData()
  : RIBConverter<ros_igtl_bridge::igtlpolydata>()
{
}

RIBConverterPolyData::RIBConverterPolyData(ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtlpolydata>(nh)
{
}

RIBConverterPolyData::RIBConverterPolyData(const char* topicPublish, const char* topicSubscribe, ros::NodeHandle *nh)
  : RIBConverter<ros_igtl_bridge::igtlpolydata>(topicPublish, topicSubscribe, nh)
{
}

int RIBConverterPolyData::onIGTLMessage(igtl::MessageHeader * header)
{

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  vtkSmartPointer<vtkPolyData>  poly = vtkSmartPointer<vtkPolyData>::New();
  
  // Create a message buffer to receive image data
  igtl::PolyDataMessage::Pointer polyDataMsg;
  polyDataMsg = igtl::PolyDataMessage::New();
  polyDataMsg->SetMessageHeader(header);
  polyDataMsg->AllocatePack();

  bool timeout(false);
  socket->Receive(polyDataMsg->GetPackBodyPointer(), polyDataMsg->GetPackBodySize(), timeout);
  int c = polyDataMsg->Unpack(1);
	
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) // if CRC check fails
    {
    std::cout<<"Unable to create PolyData from incoming POLYDATA message. Failed to unpack the message"<< std::endl;
    return 0;
    }
	
  const char* name = polyDataMsg->GetDeviceName();
	
  // Points
  igtl::PolyDataPointArray::Pointer pointsArray = polyDataMsg->GetPoints();
  int npoints = pointsArray->GetNumberOfPoints();
  if (npoints > 0)
    {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < npoints; i ++)
      {
      igtlFloat32 point[3];
      pointsArray->GetPoint(i, point);
      points->InsertNextPoint(point); 
      }
    poly->SetPoints(points);
    }
	
  // Vertices
  igtl::PolyDataCellArray::Pointer verticesArray = polyDataMsg->GetVertices();
  int nvertices = verticesArray->GetNumberOfCells();
  if (nvertices > 0)
    {
    vtkSmartPointer<vtkCellArray> vertCells = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < nvertices; i ++)
      {
      vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
			
      std::list<igtlUint32> cell;
      verticesArray->GetCell(i, cell);
      std::list<igtlUint32>::iterator iter;
      iter = cell.begin();
      vertex->GetPointIds()->SetId(i, *iter);
      vertCells->InsertNextCell(vertex);
      }
    poly->SetVerts(vertCells);
    }
	
  // Lines
  igtl::PolyDataCellArray::Pointer linesArray = polyDataMsg->GetLines();
  int nlines = linesArray->GetNumberOfCells();
  if (nlines > 0)
    {
    vtkSmartPointer<vtkCellArray> lineCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < nlines; i++)
      {
      vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
			
      std::list<igtlUint32> cell;
      linesArray->GetCell(i, cell);
      polyLine->GetPointIds()->SetNumberOfIds(cell.size());
      std::list<igtlUint32>::iterator iter;
      int j = 0;
      for (iter = cell.begin(); iter != cell.end(); iter ++)
        {
        polyLine->GetPointIds()->SetId(j, *iter);
        j++;
        }
      lineCells->InsertNextCell(polyLine);
      }
    poly->SetLines(lineCells);
    }
	
  // Polygons
  igtl::PolyDataCellArray::Pointer polygonsArray = polyDataMsg->GetPolygons();
  int npolygons = polygonsArray->GetNumberOfCells();
  if (npolygons > 0)
    {
    vtkSmartPointer<vtkCellArray> polygonCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < npolygons; i++)
      {
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
			
      std::list<igtlUint32> cell;
      polygonsArray->GetCell(i, cell);
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
    poly->SetPolys(polygonCells);
    }
	
  // Triangle Strips
  igtl::PolyDataCellArray::Pointer triangleStripsArray = polyDataMsg->GetTriangleStrips();
  int ntstrips = triangleStripsArray->GetNumberOfCells();
  if (ntstrips > 0)
    {
    vtkSmartPointer<vtkCellArray> tstripCells = vtkSmartPointer<vtkCellArray>::New();
    for(int i = 0; i < ntstrips; i++)
      {
      vtkSmartPointer<vtkTriangleStrip> tstrip = vtkSmartPointer<vtkTriangleStrip>::New();
			
      std::list<igtlUint32> cell;
      triangleStripsArray->GetCell(i, cell);
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
    poly->SetStrips(tstripCells);
    }
	
  poly->Modified();
  //Show_Polydata(poly);
  this->publisher.publish(polyDataToMsg(name,poly));

  return 1;
}


void RIBConverterPolyData::onROSMessage(const ros_igtl_bridge::igtlpolydata::ConstPtr & msg)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  msgToPolyData(msg,polydata);

  igtl::PolyDataMessage::Pointer polyDataMsg;
  polyDataMsg = igtl::PolyDataMessage::New();
  polyDataMsg->SetDeviceName(msg->name.c_str());	
  
  // points
  igtl::PolyDataPointArray::Pointer pointArray;
  pointArray = igtl::PolyDataPointArray::New();
  
  double *point;
  for (unsigned int i = 0; i <polydata->GetNumberOfPoints(); i ++)
    {
    point = polydata->GetPoint(i);
    pointArray->AddPoint(static_cast<igtlFloat32>(point[0]),static_cast<igtlFloat32>(point[1]),static_cast<igtlFloat32>(point[2]));
    }
  polyDataMsg->SetPoints(pointArray);
  
  vtkSmartPointer<vtkCellArray> cellarray =
    vtkSmartPointer<vtkCellArray>::New();
  
  igtl::PolyDataCellArray::Pointer polyArray;
  polyArray = igtl::PolyDataCellArray::New();
  
  // polys
  int npolys = polydata->GetNumberOfPolys();
  if(npolys>0)
    {
    cellarray = polydata->GetPolys();
    
    vtkSmartPointer<vtkIdList> IdList =
      vtkSmartPointer<vtkIdList>::New();
    
    cellarray->InitTraversal();
    while (cellarray->GetNextCell(IdList))
      {
      std::list<igtlUint32> cell;
      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
        {
        cell.push_back(IdList->GetId(i));
        }
      polyArray->AddCell(cell);
      }
    polyDataMsg->SetPolygons(polyArray);
    }
  
  // vertices
  int nverts = polydata->GetNumberOfVerts();
  if(nverts>0)
    {
    cellarray = polydata->GetVerts();
    
    vtkSmartPointer<vtkIdList> IdList =
      vtkSmartPointer<vtkIdList>::New();
    
    cellarray->InitTraversal();
    while (cellarray->GetNextCell(IdList))
      {
      std::list<igtlUint32> cell;
      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
        {
        cell.push_back(IdList->GetId(i));
        }
      polyArray->AddCell(cell);
      }
    polyDataMsg->SetVertices(polyArray);
    }	

  // lines
  int nlines = polydata->GetNumberOfLines();
  if(nlines>0)
    {
    cellarray = polydata->GetLines();
		
    vtkSmartPointer<vtkIdList> IdList =
      vtkSmartPointer<vtkIdList>::New();
		
    cellarray->InitTraversal();
    while (cellarray->GetNextCell(IdList))
      {
      std::list<igtlUint32> cell;
      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
        {
        cell.push_back(IdList->GetId(i));
        }
      polyArray->AddCell(cell);
      }
    polyDataMsg->SetLines(polyArray);
    }
	
  // strips
  int nstrips = polydata->GetNumberOfStrips();
  if(nstrips>0)
    {
    cellarray = polydata->GetStrips();
		
    vtkSmartPointer<vtkIdList> IdList =
      vtkSmartPointer<vtkIdList>::New();
		
    cellarray->InitTraversal();
    while (cellarray->GetNextCell(IdList))
      {
      std::list<igtlUint32> cell;
      for (int i = 0; i < IdList->GetNumberOfIds(); i++)
        {
        cell.push_back(IdList->GetId(i));
        }
      polyArray->AddCell(cell);
      }
    polyDataMsg->SetTriangleStrips(polyArray);
    }
  // pack and send
  polyDataMsg->Pack();
	
  socket->Send(polyDataMsg->GetPackPointer(), polyDataMsg->GetPackSize());
}


//----------------------------------------------------------------------
void RIBConverterPolyData::msgToPolyData( const ros_igtl_bridge::igtlpolydata::ConstPtr& msg, vtkSmartPointer<vtkPolyData> polydata)
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


ros_igtl_bridge::igtlpolydata RIBConverterPolyData::polyDataToMsg(const char* name, vtkSmartPointer<vtkPolyData> polydata )
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

