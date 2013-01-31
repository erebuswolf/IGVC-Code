/**
*	Display.cpp
*	Author: Jesse Fish
*
*/

#include "Display.h"
#include <string>
#include <math.h>
#include <stdio.h>
using namespace std;
using namespace cv;


Display::Display(WorldModel* model){	
  this->model=model;

  
  planner=NULL;
  windata.xmin=0;
  windata.ymin=0;
  windata.width=600;
  windata.height=600;
  windowname="WorldModelImage";
  windowname2="PlannerImage";
  visionwindow="VisionImage";
  cout<<"creating window"<<endl;
  // Create a window in which the captured images will be presented
  cvNamedWindow(windowname.c_str(), CV_WINDOW_AUTOSIZE );
  cvNamedWindow(windowname2.c_str(), CV_WINDOW_AUTOSIZE );
  
  if(model->useVision){ 
  	cvNamedWindow(visionwindow.c_str(), CV_WINDOW_AUTOSIZE );
  }
  displayImage=cvCreateImage(cvSize(windata.width,windata.height),8,3);	
  displayImage2=cvCreateImage(cvSize(windata.width,windata.height),8,3);
}


Display::~Display(){
  cvDestroyWindow( windowname.c_str());
  cvDestroyWindow(windowname2.c_str() );
  cvReleaseImage(&displayImage);
}


void Display::displayModel(){
  
  //cout<<"displaying the model"<<endl;
  CvRect roi=model->getRegionCentered(windata.width,windata.height);

  roi = model->correctROI(roi);

  cvSetImageROI(model->worldModel,roi);
  
  cvCvtColor(model->worldModel,displayImage,CV_GRAY2RGB);
/* if(planner!=NULL){ 
 	cvResetImageROI(planner->planImage);
	cvSetImageROI(planner->planImage,roi);
	cvCvtColor(planner->planImage,displayImage2,CV_GRAY2RGB);
  	cvResetImageROI(planner->planImage);
}*/

  //this command works entirely based on the fact that the robot is always centered in the image
  //this might be a magic number that we want to move to a better place
  float heading_length=20;
  CvPoint origin=cvPoint(windata.width/2,windata.height/2);
  cvCircle(displayImage,origin,4,cvScalar(0,255,0),-1,8);
  cvLine(displayImage,origin,cvPoint(cos(-(model->state.crioRotRad)-3.1415/2)*heading_length+origin.x,sin(-(model->state.crioRotRad)-3.1415/2)*heading_length+origin.y),cvScalar(0,0,255),2);
  
  
  cvShowImage( windowname.c_str(),displayImage);
  
  if(planner!=NULL){
//	cvShowImage( windowname2.c_str(),displayImage2);
  }
  
  cvResetImageROI(model->worldModel);
  
  /*if(model->useVision){
	if(model->worldModelCombined!=NULL){
	//	cvSetImageROI(model->completePlanWorldMap ,roi);
	  //model->worldModelVisionAdd
//		cvShowImage( visionwindow.c_str(),model->completePlanWorldMap);
	 
	//	cvResetImageROI(model->completePlanWorldMap);
	}
	else{
	  printf("null world model combined image.\n");
	}
  }*/
}

