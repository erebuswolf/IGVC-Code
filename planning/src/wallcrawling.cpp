/**
*	wallcrawling.cpp
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
#include "observermode.h"
#include "Display.h"
#include "WallCrawler.h"

void wallcrawling(){

	
	Lidar lidar;
	lidar.init("lidar/lidarconfig.conf");
	LidarManager lidarManager;
	lidarManager.lidar=&lidar;
	lidarManager.init("lidar/lidarpositionconfig.conf");

	WorldModel model;
	model.init("visionworldmodelconfig.conf");
	model.lidar=&lidarManager;
	
	Network network;
	
	model.network=&network;

	
	cout<<"creating planner\n";
	Planner *wallcrawlingPlanner=new WallCrawler(.7,.3,&model,&network);
	cout<<"planner made\n";
	
	//vision setup
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
	
	Display display=Display(&model);
	display.planner=wallcrawlingPlanner;
	
	//start timer
	TimeKeeper::start_time();

	printf("made all objects\n");
	int key = 0;	
	long start=0;
	try {
		while((key&255)!=27){
		//	start=TimeKeeper::GetTime();
			wallcrawlingPlanner->updatePlanner();
			display.displayModel();
			key=cvWaitKey(3);

	//		cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
	//		cout<<TimeKeeper::GetTime()-start<<endl;
		}
	}
	catch(...) {
    cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
  delete wallcrawlingPlanner;
  
}
