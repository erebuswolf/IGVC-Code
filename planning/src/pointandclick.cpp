/**
*	pointandclick.cpp
*	Author: Jesse Fish
*
*/

#include "Lidar.h"
#include "LidarManager.h"
#include "TimeKeeper.h"
#include "Planner.h"
#include "WorldModel.h"
#include <stdio.h>
#include <iostream>
#include "NetworkStack.h"
#include "pointandclick.h"
#include "Display.h"
#include "PointAndClickDisplay.h"
#include "Camera.h"
#include "CameraManager.h"
#include "cv.h"
#include "highgui.h"


using namespace std;
using namespace cv;

void pointandclick(){

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
	printf("made network\n");
	
	Camera a;
   	a.init(0);
	a.loadCalib("vision/calibration/Intrinsic.xml","vision/calibration/Distortion.xml","vision/calibration/H.xml");
	CameraManager manager;
   	manager.init();
   	manager.addCam(&a);
   	manager.startThread();
   	
	printf("made camera\n");
	
   	VisionInterpreter vision= VisionInterpreter(&manager);
   	printf("pre init\n");
   	vision.init("vision/visioninterpreter.conf");
   	printf("post init\n");
   	vision.manager=&manager;
   	model.vision=&vision;
   	
	printf("made vision interpreter\n");
	
	PointAndClickDisplay display=PointAndClickDisplay(&model);

	printf("made all objects\n");
	//start timer
	TimeKeeper::start_time();
	int key = 0;	
	long start=0;
	try {
		while((key&255)!=27){
			start=TimeKeeper::GetTime();
			cout<<"update model\n";
			model.updateModel();
			cout<<"update disp\n";
			display.displayModel();
			key=cvWaitKey(3);
			  
		}
			
		//	cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
	//		cout<<TimeKeeper::GetTime()-start<<endl;
	}
	catch(...) {
    cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
}

