/**
*	doGPS.cpp
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

#include <math.h>
#include "BugPlanner.h"
#include <iostream>
#include <fstream>
#include "ThreadGPSPlanner.h"
#include "TrajectoryGenerator.h"
#include "doGPS.h"

enum plannerstate{bugging_to_goal,planning,bugging_to_path};

void doGPS(){
  plannerstate planstate=bugging_to_goal;
  
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
  
 // Display display=Display(&model);
  cout<<"creating planner\n";
  
  TrajectoryGenerator trajgen(0.8f);
  Planner* planner=new ThreadGPSPlanner(&model, &trajgen,0.5f);
  
 // display.planner=planner;
  
  //bug planner for backup
  BugPlanner *bugPlanner=new BugPlanner(.3,.3,&model,&network);
  
  ((ThreadGPSPlanner*)planner)->loadGoals("gpspoints.txt");
  
  model.planner_world=planner->planImage;
  
  //start timer
  TimeKeeper::start_time();
  
  printf("made all objects\n");
  int key = 0;	
  long start=0;
  
  int closest_path_index=0;
  
  int wait_for_planner=20;
  int num_updates=0;
  bool endofpath=false;
  while((key&255)!=27){
	try {
	  start=TimeKeeper::GetTime();
	  model.updateModel();
	  
	  if(wait_for_planner>num_updates){
		num_updates++;
	  }else if(((ThreadGPSPlanner*)planner)->killthread){
		//start planning thread
		((ThreadGPSPlanner*)planner)->startThread();
	  }
	  
	  int goalIndex=((ThreadGPSPlanner*)planner)->goalIndex;
	  cout<<"heading to goal "<<goalIndex<<endl;
		  //find state
	  if( ((ThreadGPSPlanner*)planner)->goalIndex < ((ThreadGPSPlanner*)planner)->goals.size()){
		
		if(trajgen.gotNewPath()){
		  planstate= bugging_to_path;
		  //find closest goal on path and bug to it
		  float distance_to_point_sqred;
		  float diffx=trajgen.path[0].x- model.state.crioPosition.x;
		  float diffy=trajgen.path[0].y- model.state.crioPosition.y;
		  distance_to_point_sqred=diffx*diffx+diffy*diffy;
		  
		  for(int i=1;i<trajgen.path.size();i++){
			diffx=trajgen.path[i].x- model.state.crioPosition.x;
			diffy=trajgen.path[i].y- model.state.crioPosition.y;
			float temp_dist=diffx*diffx+diffy*diffy;
			if(temp_dist<distance_to_point_sqred){
			  distance_to_point_sqred=temp_dist;
			  closest_path_index=i;
			}
		  }
		  cout<<"set the bug goal to path"<<endl;
		  bugPlanner->setGoal(trajgen.path[closest_path_index].x,trajgen.path[closest_path_index].y);
		  trajgen.pathIndex=closest_path_index;
		}else if(!endofpath){
		  planstate=bugging_to_goal;
		if( bugPlanner->goalPoint.x!=((ThreadGPSPlanner*)planner)->goals[goalIndex].x || bugPlanner->goalPoint.y!=((ThreadGPSPlanner*)planner)->goals[goalIndex].y){ 
		  bugPlanner->setGoal(((ThreadGPSPlanner*)planner)->goals[goalIndex].x,((ThreadGPSPlanner*)planner)->goals[goalIndex].y);
		  }

		}
		if(model.inDanger){
			planstate=bugging_to_goal;
			bugPlanner->setGoal(((ThreadGPSPlanner*)planner)->goals[goalIndex].x,((ThreadGPSPlanner*)planner)->goals[goalIndex].y);
			bugPlanner->updatePlanner();
		}
		
		
		if(planstate== planning){
		  cout<<"planning wee "<<endofpath<<endl;
		  endofpath=trajgen.updateRobot(&model,10);
		}
		else if(planstate==bugging_to_path){
		 cout<<"bugging to path"<<endl;
		  //check if we are on the path, if so, bind to it
		 float diffx=trajgen.path[closest_path_index].x- model.state.crioPosition.x;
		 float diffy=trajgen.path[closest_path_index].y- model.state.crioPosition.y;
		  
		  float distance_to_point_sqred=diffx*diffx+diffy*diffy;
		  //TODO:make this not a constant
		  if(sqrt(distance_to_point_sqred)<0.5){
			planstate=planning;
			endofpath=trajgen.updateRobot(&model,6);
		  }
		  else{
			bugPlanner->updatePlanner();
		  }
		}else{
			cout<<"bugging"<<endl;
			bugPlanner->updatePlanner();
		}
	  }
	  else{
		cout<<"hit all goals"<<endl;
		 network.sendWarning();

	  }
//	  display.displayModel();
//	  key=cvWaitKey(3);
	  //cout<<model.state.crioPosition.x<<" "<<model.state.crioPosition.y<<" "<<model.state.crioRot<<endl;
	  cout<<TimeKeeper::GetTime()-start<<endl;
	}
	catch(...) {
	  cerr << "we are catching an uncaught exception. bad things happened" << endl;
	}
  }
  network.sendWarning();
}
