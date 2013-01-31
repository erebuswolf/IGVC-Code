/**
*	recorder.cpp
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
#include "observermode.h"
#include "Display.h"
#include <fstream>


void recorder(){

	ofstream logfile;
	logfile.open("logfile.txt");

	ofstream lidarLogfile;
	lidarLogfile.open("lidarlogfile.txt");
	
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
	
   char buffer [50];
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

	//start timer
	TimeKeeper::start_time();

	printf("made all objects\n");
	int key = 0;	
	long start=0;
	try {
		while((key&255)!=27){
			start=TimeKeeper::GetTime();
			model.updateModel();
			display.displayModel();
			key=cvWaitKey(3);

			//write a timestamp to the logfile
			logfile<<start<<endl;
			//write the crio state to the logfile
			logfile<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRotRad<<endl;
			//write the lidar data to the logfile
			
			lidarLogfile<<start<<endl;
			lidarLogfile<<lidar.num_values<<endl;
			for(int i=0;i<lidar.num_values;i++){
				lidarLogfile<<lidar.values[i]<<" ";
			}
			lidarLogfile<<endl;
		
			sprintf (buffer, "output/output%.7ld.bmp",start);
			if(!cvSaveImage(buffer,a.image,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
			cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRotRad<<endl;
			cout<<TimeKeeper::GetTime()-start<<endl;
			
			if((key&255) ==32){
				if(!cvSaveImage("output/vision map.jpg",model.worldModelVision,0)) printf("Could not save");
				else printf("picture taken!!!\n");
		  }
		}
	}
	catch(...) {
    cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
  lidarLogfile.close();
	logfile.close();
}

