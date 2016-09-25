#ifndef ShowPolyData_H
#define ShowPolyData_H

// VTK
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkDataSetAttributes.h>
#include <vtkPolyData.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>

//----------------------------------------------------------------------
void Show_Polydata(vtkSmartPointer<vtkPolyData> polydata)
{
	vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	#if VTK_MAJOR_VERSION <= 5
		meshMapper->SetInputConnection(polydata->GetProducerPort());
	#else
		meshMapper->SetInputData(polydata);
	#endif
	vtkSmartPointer<vtkActor> meshActor = vtkSmartPointer<vtkActor>::New();
	meshActor->SetMapper(meshMapper);

	vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	#if VTK_MAJOR_VERSION <= 5
		glyphFilter->SetInputConnection(polydata->GetProducerPort());
	#else
		glyphFilter->SetInputData(polydata);
	#endif
	glyphFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pointMapper->SetInputConnection(glyphFilter->GetOutputPort());

	vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
	pointActor->GetProperty()->SetColor(1,0,0);
	pointActor->GetProperty()->SetPointSize(0.05);
	pointActor->SetMapper(pointMapper);
	
	vtkSmartPointer<vtkAxesActor> axesactor = vtkSmartPointer<vtkAxesActor>::New();
                      
	axesactor->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axesactor->SetShaftTypeToCylinder();
	axesactor->SetCylinderRadius(0.02);
	axesactor->SetXAxisLabelText("X");
	axesactor->SetYAxisLabelText("Y");
	axesactor->SetZAxisLabelText("Z");
	axesactor->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	axesactor->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	axesactor->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	axesactor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(17);
	axesactor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(17);
	axesactor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(17);
	axesactor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
	axesactor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
	axesactor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->BoldOff();
	axesactor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
	axesactor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
	axesactor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetItalic(0);
	axesactor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
	axesactor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
	axesactor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetShadow(0);
	axesactor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1.0, 0.0,
																	  0.0);
	axesactor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 1.0,
																	  0.0);
	axesactor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0,
																	  1.0);
	//actor->SetUserTransform(trans);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);
	
	renderer->AddActor(meshActor);
	renderer->AddActor(pointActor);
	renderer->AddActor(axesactor);
	renderer->SetBackground(1, 1, 1); 
	//renderer->SetBackground(.3, .6, .3); 
	renderWindow->Render();
	renderWindowInteractor->Start();
}
#endif
