/**
*
*     PostProcess.cpp
*     Author: Aaron Deal
*
**/

#include "WorldModel.h"
#include "Camera.h"
#include "CameraManager.h"
#include "VisionInterpreter.h"
#include "Display.h"
#include "Line.h"
#include "Breadth.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "Lidar.h"
#include "LidarManager.h"
#include "PostProcess.h"

void postprocess()
{
	float meterToPixel = 0.05;

	ifstream config_file("postprocess.conf");
	string path;
	long timestamp;
	if(config_file.is_open()){
		//read in path to directory
		config_file>>path;
		config_file.ignore(300, '\n');
		//read in timestamp on first image
		config_file>>timestamp;
		//close the file
		config_file.close();
	}
	else{
		cerr<<"Unable to open file."<<endl;
	}
	
	WorldModel model;
	model.init("postworldmodelconfig.conf");
	
	Display display=Display(&model);
	
	char image_dir[500];
	sprintf(image_dir,"%s/output/",path.c_str());
	Camera a(image_dir, timestamp);
   	a.init(0);
	cout<<"Camera is made"<<endl;
	
	char intrinsic[500];
	sprintf(intrinsic, "%s/Intrinsic.xml", path.c_str());
	char distortion[500];
	sprintf(distortion, "%s/Distortion.xml", path.c_str());
	char H[500];
	sprintf(H, "%s/H.xml", path.c_str());
	a.loadCalib(intrinsic,distortion,H);
	cout<<"calibration parameters are loaded"<<endl;
	
	
	CameraManager manager;
   	manager.init();
   	manager.addCam(&a);
   	
	VisionInterpreter vision= VisionInterpreter(&manager);
   	vision.init("vision/visioninterpreter.conf");
   	vision.manager=&manager;
   	
   	model.vision=&vision;
   	
	Breadth breadth;

	/*sprintf(image_dir,"%s/lidarlogfile.txt",path.c_str());
	Lidar lidar(image_dir);
	lidar.init("lidar/lidarconfig.conf");
	LidarManager lidarManager;
	lidarManager.lidar=&lidar;
	lidarManager.init("lidar/lidarpositionconfig.conf");

	model.lidar=&lidarManager;*/

   	
   	
   	
   	Line line;
   	line.init("line.conf");
   	line.model = &model;
   	Tire tires[2];
   	tires[0].index = tires[1].index = 0;
   	CvPoint initial, robot, start, end;
   	int heading;
   	IplImage *copy = cvCreateImage( cvGetSize(model.worldModelVision), IPL_DEPTH_8U, 1);
   	IplImage *sub = cvCreateImage( cvGetSize(model.worldModelVision), IPL_DEPTH_8U, 1);
   	IplImage *map = cvCreateImage( cvGetSize(model.worldModelVision), IPL_DEPTH_8U, 1);
   	IplImage *final = cvCreateImage( cvGetSize(model.worldModelVision), IPL_DEPTH_8U, 1);
   	int key = 0;
   	int change = 0;
   	//CvPoint display;
	//CvPoint robot;
	//CvPoint initial;
	//double heading;
	double init_heading;
	
	cvSetZero(model.worldModelVisionSub);
	cout<<"Starting post processing\n";
   	while(key!=27)
   	{
   		if(model.updateModel()==-1)
   			break;
   		cout<<"Updated Model\n";
   		display.displayModel();
   		
   		//breadth.buildInitialMap();
   		
   		if(change==0){
	   		initial.x = (int)(model.state.crioPosition.x*(1.0/meterToPixel));
	   		initial.y = (int)(model.state.crioPosition.y*-(1.0/meterToPixel));
   			init_heading = model.state.crioRotRad*180./3.14159;
   			line.setInitialHeading(init_heading);
   			change++;
   		}
   		
   		robot.x = (int)(model.state.crioPosition.x*(1.0/meterToPixel)) + model.translation.x;
   		robot.y = (int)(model.state.crioPosition.y*-(1.0/meterToPixel)) + model.translation.y;
   		line.qualsLineFinder(model.worldModelVision, robot); 
		
		cvWaitKey(20);
		
   		/*if(change==0){
	   		initial.x = (int)(model.state.crioPosition.x*(1.0/meterToPixel));
	   		initial.y = (int)(model.state.crioPosition.y*-(1.0/meterToPixel));
   			init_heading = model.state.crioRotRad*180./3.14159;
   			line.setInitialHeading(init_heading);
   			cout<<init_heading<<endl;
   			change++;
   		}
   		start.x = initial.x + model.translation.x;
   		start.y = initial.y + model.translation.y;
   		
   		robot.x = (int)(model.state.crioPosition.x*(1.0/meterToPixel)) + model.translation.x;
   		robot.y = (int)(model.state.crioPosition.y*-(1.0/meterToPixel)) + model.translation.y;
   		heading = model.state.crioRotRad*180./3.14159;*/
   		
   		//cvOr(model.worldModelVision, final, final);
   		//cvAdd(final, model.worldModelVisionAdd, final);
   		//cvSub(final, model.worldModelVisionSub, final);
   		
   		//cvSetImageROI(final, cvRect(robot.x-250, robot.y-250, 500, 500));
   		//cvSetImageROI(copy, cvRect(robot.x-250, robot.y-250, 500, 500));
   		
   		//cvAdd(copy, final, final);
   		//cvResetImageROI(copy);
   		//cvResetImageROI(final);
   		
   		//cvShowImage("Orig",final);
   		//while((cvWaitKey(10)&255)!=32);
   		/*cvAdd(model.worldModelVision, model.worldModelVisionAdd, final);
   		
   		cvSetImageROI(final, cvRect(robot.x-250, robot.y-250, 500, 500));
   		cvShowImage("combined", final);
   		cvWaitKey(4);
   		cvResetImageROI(final);
   		vector<CvPoint> midpoints = line.main(final, map, robot, heading);
   		cvShowImage("map output", map);
   		
   		//cvSetImageROI(final, cvRect(robot.x-250, robot.y-250, 500, 500));
   		//cvShowImage("Line Memory", final);
   		//cvResetImageROI(final);
   		
   		cvSetImageROI(map, cvRect(robot.x-100, robot.y-100, 200, 200));
   		CvScalar total = cvSum(map);
   		cvResetImageROI(map);
   		
   		//cvSet(sub, cvScalar(10));
   		//cvSub(model.worldModelVisionSub, sub, model.worldModelVisionSub);
   		if( midpoints.size() != 0 ){
   			cvSet(sub, cvScalar(255));
   			cvConvertScale(sub, sub, 30./255);
   			//subtract constant image
   			cvSub(model.worldModelVisionAdd, sub, model.worldModelVisionAdd);
   			
   			cvSet(sub, cvScalar(255));
   			cvSub(sub, map, sub);
   			cvConvertScale(sub, sub, 15./255);
   			cvSub(model.worldModelVision, sub, model.worldModelVision);
   			//cvCircle(model.worldModelVisionAdd, midpoints[midpoints.size()-1], 2, cvScalar(150), 1);
	   		cvCircle(map, midpoints[midpoints.size()-1],1.3*line.meterToPixel, cvScalar(0), 3*line.meterToPixel);
	   		cvConvertScale(map, map, 100./255);
	   		cvAdd(model.worldModelVisionAdd, map, model.worldModelVisionAdd);
   		}
   		cvSetImageROI(model.worldModelVisionAdd, cvRect(robot.x-250, robot.y-250, 500, 500));
   		//cvSetImageROI(model.worldModelVisionAdd, cvRect(robot.x-250, robot.y-250, 500, 500));
   		cvShowImage("Lines", model.worldModelVisionAdd);
   		//cvShowImage("Add", model.worldModelVisionAdd);
   		//cvResetImageROI(model.worldModelVisionAdd);
   		cvResetImageROI(model.worldModelVisionAdd);*/
   		key = cvWaitKey(50);
   		key = key&255;
   		
   	}
   	cvResetImageROI(model.worldModelVision);
   	cvSaveImage("worldmodelvision.jpeg", model.worldModelVision);
   	cvWaitKey(0);
}
