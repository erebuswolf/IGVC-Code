/**
*	qualifier.cpp
*	Author: Jesse Fish
*
*/

#include "Lidar.h"
#include "LidarManager.h"
#include "TimeKeeper.h"
#include "Planner.h"
#include "WorldModel.h"
#include "Camera.h"
#include "CameraManager.h"
#include "VisionInterpreter.h"
#include <stdio.h>
#include <iostream>
#include "NetworkStack.h"
#include "Display.h"
#include "TrajectoryGenerator.h"
#include "ThreadGPSPlanner.h"
#include "Line.h"

#include <iostream>
#include <fstream>
#include "qualifier.h"
#include <math.h>
#include "simplesteering.h"

void qualifier(){
  

	
	WorldModel model;
	model.init("visionworldmodelconfig.conf");
	Network network;
	model.network=&network;
	//let the network get a few data points in for our position
	sleep(1);
	

	Camera a;
   	a.init(0);
	a.loadCalib("vision/calibration/Intrinsic.xml","vision/calibration/Distortion.xml","vision/calibration/H.xml");
	CameraManager manager;
   	manager.init();
   	manager.addCam(&a);
   	manager.startThread();
   	VisionInterpreter vision= VisionInterpreter(&manager);
   	vision.init("vision/visioninterpreter.conf");
   	vision.manager=&manager;
   	model.vision=&vision;
   	
	Lidar lidar;
	lidar.init("lidar/lidarconfig.conf");
	LidarManager lidarManager;
	lidarManager.lidar=&lidar;
	lidarManager.init("lidar/lidarpositionconfig.conf");
	model.lidar=&lidarManager;
	

	Line line;
	line.init("line.conf");
	line.model = &model;
	
	Display display=Display(&model);
	cout<<"creating planner\n";
	//TODO:generate a planner to put here
//	TrajectoryGenerator trajgen(0.6f);
	//Planner* planner=new ThreadGPSPlanner(&model, &trajgen,0.5f);
//	  ((ThreadGPSPlanner*)planner)->loadGoals("gpspoints.txt");
  
//	display.planner=planner;
	
	//((GPSPlanner*)planner)->loadGoals("gpspoints.txt");
	
	//model.planner_world=planner->planImage;
	
	cout<<"planner made\n";
	
	//TODO: put the planner image here
	//this is where the planner image would be put if we had one
	//display.planner=bugPlanner;
	
	CvPoint robot;
	//vector<CvPoint2D32f> goals;
		
	//start timer
	TimeKeeper::start_time();
	srand ( time(NULL) );

	printf("made all objects\n");
	int key = 0;	
	long start=0;
	CvPoint2D32f currentGoal;
	int sillysize=100;
	IplImage * mask=cvCreateImage(cvSize(sillysize,sillysize),8,1);
	bool setgoal=false;
	currentGoal.x=0;
	currentGoal.y=0;
	while((key&255)!=27){
	try {
			cout<<"starting\n";
			start=TimeKeeper::GetTime();
			
			cout<<"update model\n";
			model.updateModel();
			
			//try to find the lines
			robot.x = (model.state.crioPosition.x*(1.0/model.meterToPixel)) + model.translation.x;
			robot.y = (model.state.crioPosition.y*(-1.0/model.meterToPixel)) + model.translation.y;
			
			
			line.main(model.worldModelVision, model.worldModelVisionAdd,robot, model.state.crioRot);
			
			float diffx=currentGoal.x-model.state.crioPosition.x;
			float diffy=currentGoal.y-model.state.crioPosition.y;
			
			if(!setgoal||sqrt(diffx*diffx+diffy*diffy)){
			  network.sendWarning();
			  cout<<"at goal"<<endl;
			  //we are at goal, pick another one'
			  bool looking=true;
			  CvRect roi=model.getRegionCentered(sillysize,sillysize);
			  cvSetImageROI(model.completePlanWorldMap,roi);
			  float heading=model.state.crioRotRad;
			  float offset=10*3.14/180;
			  CvPoint possible_goal_pixel;
			  CvPoint center;
			  center.x=sillysize/2;
			  center.y=sillysize/2;
			  int count=100;
			  while(looking && count>0){
			 	 count--;
				cvSetZero(mask);
				
				heading=(rand()*offset)/RAND_MAX;
				possible_goal_pixel.x=cos(heading)*20+sillysize/2;
				possible_goal_pixel.y=sin(heading)*20+sillysize/2;
				
				cvLine(mask, center, possible_goal_pixel, cvScalar(255), 1);
				//cvShowImage("bas",mask);
				//cvAnd(model.completePlanWorldMap,mask,mask);
				//cvShowImage("bas1",mask);
				//cvShowImage("bas22",model.completePlanWorldMap);

				//cvWaitKey(0);
				
				CvScalar total=cvSum(mask);
//				cout<<total.val[0]<<endl;
				if(total.val[0]<4000){
				  looking=false;
				  currentGoal=model.pixeltometer(possible_goal_pixel.x+roi.x+roi.width/2,possible_goal_pixel.y+roi.y+roi.width/2);
				}
				else{
				  offset+=.1;
				}
			  }
			  cvResetImageROI(model.completePlanWorldMap);
			  
			  setgoal=true;
			}
			else{
			 //keep steering to the current goal 
			 
			  AngularRateHolder temp =simplesteering( currentGoal.x,currentGoal.y, model.state,.5,.7,3/3.1415,0.9,3.1415/6);
			  printf("sending vel command vel %f, omega %f\n", temp.vel,temp.omega);
			  
			  network.angularRateSpeedCommand( temp.omega,temp.vel);
			}
			display.displayModel();
	
			cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
			cout<<TimeKeeper::GetTime()-start<<endl;
		}
	
	catch(...) {
    cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
  	key=cvWaitKey(3);
  }
  network.sendWarning();
}
