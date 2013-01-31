/**
*	lidarlocalizorapp.cpp
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
#include "LidarLocalizer.h"


void lidarlocalizorapp(){
  printf("starting\n");
 
  Lidar lidar;
  lidar.init("lidar/lidarconfig.conf");
  
  LidarManager lidarManager;
  lidarManager.lidar=&lidar; 
  
  lidarManager.init("lidar/lidarpositionconfig.conf");
  
  
  LidarLocalizer localizer;
  localizer.loadPresetMap();
  
  WorldModel model;
  model.init("indoorlidarconf.conf");
  model.lidar=&lidarManager;
  model.lidarLocalizer=&localizer;
  
  Network network;  
  model.network=&network;
  
	Display display=Display(&model);
  
  //start timer
  TimeKeeper::start_time();

  printf("made all objects\n");
  int key = 0;	
  long start=0;
  double x;
  double y;
  try {
	while((key&255)!=27){

	  start=TimeKeeper::GetTime();
	 // printf("loop start\n");
	  
	  model.updateModel();
	//  localizer.estimatePosition(&state,&scanpoints,fake_scan_point_count);

	//  printf("model updated\n");
	  //display.displayModel();
      key=cvWaitKey(15);
	  
	  cout<<"position "<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
	  cout<<"time elapsed "<<(TimeKeeper::GetTime()-start)/1000000.0<<endl;
	
	}
  }
  catch(...) {
	cerr << "we are catching an uncaught exception. bad things happened" << endl;
  }
  
}
