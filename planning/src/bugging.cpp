/**
*	bugging.cpp
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

#include <iostream>
#include <fstream>
#include "BugPlanner.h"
#include "bugging.h"
#include "convertgps.h"

void bugging(){

	//load coordinates
	int pointIndex=0;
	float xpoints [10];
	float ypoints [10];
int number_of_points;
	ifstream config_file ("gpspoints.txt");
	 if (config_file.is_open())
	{config_file>>number_of_points;
	  for (int i=0;i<number_of_points;i++)
	  {
		  config_file>>xpoints[i];				
		  config_file>>ypoints[i];
		  CvPoint2D32f temppoint=convertgps(xpoints[i],ypoints[i]);
		  xpoints[i]=temppoint.x;
		  ypoints[i]=temppoint.y;
		 // float temp=(xpoints[i]-42.6778389)*111063.5799731039; 
		//  xpoints[i]=(ypoints[i] -(-83.1952306))*81968 ;
		//  ypoints[i]=temp;
		  cout<<"points "<<xpoints[i]<<" "<<ypoints[i]<<endl;
	  }
	}
	else cerr << "Unable to open gpspoints file" << endl; 
	cout<<"points loaded\n";

	Lidar lidar;
	lidar.init("lidar/lidarconfig.conf");
	LidarManager lidarManager;
	lidarManager.lidar=&lidar;
	lidarManager.init("lidar/lidarpositionconfig.conf");

	WorldModel model;
	model.init("worldmodelconfig.conf");
	model.lidar=&lidarManager;
	
	Network network;
	
	model.network=&network;
	//let the network get a few data points in for our position
	sleep(1);
	
	Display display=Display(&model);
	
	cout<<"creating planner\n";
	BugPlanner *bugPlanner=new BugPlanner(.5,.3,&model,&network);
	cout<<"planner made\n";
	
	
	display.planner=bugPlanner;
	
	//start timer
	TimeKeeper::start_time();

	printf("made all objects\n");
	int key = 0;	
	long start=0;
/*	double x;
	double y;
	cout<<"enter a goal x\n";
	cin>>x;
	cout<<"enter a goal y\n";
	cin>>y;*/	
	while((key&255)!=27){
	try {
			start=TimeKeeper::GetTime();
			bugPlanner->updatePlanner();
			display.displayModel();
			key=cvWaitKey(3);
			if(bugPlanner->atGoal||pointIndex==0){
				if(pointIndex==0)
				{
					bugPlanner->setGoal(xpoints[pointIndex],ypoints[pointIndex]);
					pointIndex++;
					
				}
		
				else{
					//ask for new goal position in this if statement
					cout << "found point"<<pointIndex<<" going to next point" << endl; 
					if(pointIndex<number_of_points){
					  bugPlanner->setGoal(xpoints[pointIndex],ypoints[pointIndex]);
					  pointIndex++;
					
					  }
					else{
						break;
					}
				}
			}

	//		cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
		//	cout<<TimeKeeper::GetTime()-start<<endl;
		}
	
	catch(...) {
    cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
  }
  network.sendWarning();
  delete bugPlanner; 
}
