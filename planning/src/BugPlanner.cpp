/**
*	BugPlanner.cpp
*	Author: Jesse Fish
*
*	bug planner
*
*/

#include "BugPlanner.h"
#include <math.h>
#include "simplesteering.h"
#include <stdio.h>

BugPlanner::BugPlanner(double left_wall_dist, double left_wall_leeway,WorldModel* model, Network* network): WallCrawler (left_wall_dist, left_wall_leeway, model,  network){
disengage_count=0;
  disengaging=false;
  //distance in meters that is close enough to the goal
  goalLeeway=.3;
  
  goalPathMask=cvCloneImage(model->worldModel);
  cvSetZero(goalPathMask);
  atGoal=false;
  robotMask=cvCreateImage(cvSize(maskWidth,maskHeight),8,1);
  debugmask=cvCloneImage(goalPathMask);
  cvSetZero(robotMask);
  //draw a circle for the robot in the middle
  cvCircle(robotMask,cvPoint(maskWidth/2,maskHeight/2),(robot_length/model->meterToPixel)/(2.5) ,cvScalar(255),-1,8);

  model->planner_world=goalPathMask;
  
  planImage=debugmask;

  //robotMask;
}

BugPlanner::~BugPlanner(){
  cvReleaseImage(&goalPathMask);
  cvReleaseImage(&robotMask);
  
}
void BugPlanner::setGoal(double x, double y){
  goalPoint.x=x;
  goalPoint.y=y;
  engaged=false;
  atGoal=false;
  drawGoalPath();
}

//check if our current position intersects with the path
bool BugPlanner::checkOnPath(){
  //using the model find a decent way to crawl along a wall 
//	cvShowImage("blah",goalPathMask);
  
  CvRect roi=model->getRegionCentered(maskWidth,maskHeight);
//  cout<<roi.x<<" "<<roi.y<<" "<<maskWidth<<" "<<maskHeight<<endl;
  cvSetImageROI(goalPathMask,roi);
  
  
  cvAnd(goalPathMask,robotMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar forwardSum=cvSum(mathMask);
  
  cvSetImageROI(debugmask,roi);
  cvOr(debugmask,robotMask,debugmask);
  
  cvResetImageROI(goalPathMask);
  cvResetImageROI(debugmask);
  
  if(forwardSum.val[0]> 0){
	return true;
  }
  return false;
}

///draw a line from the robot to the goal point in a image
void BugPlanner::drawGoalPath(){
  
  //clear the line mask
  cvSetZero(goalPathMask);
  cvResetImageROI(goalPathMask);  
  Point_<double> A;
  A.x=goalPoint.x*(1.0/model->meterToPixel)+model->translation.x;
  A.y=goalPoint.y*-(1.0/model->meterToPixel)+model->translation.y;

  Point_<double> B; 
  B.x=model->state.crioPosition.x*(1.0/model->meterToPixel)+model->translation.x;
  B.y=model->state.crioPosition.y*-(1.0/model->meterToPixel)+model->translation.y; 
  
  cvLine(goalPathMask, A, B,cvScalar(255),2);
  
}

bool BugPlanner::updatePlanner(){
  model->updateModel();
  cvResetImageROI(goalPathMask);
  cvResetImageROI(planImage); 
  cvCopy(goalPathMask,planImage);
  
  //update the state of the wall crawler regardless so we can see if a wall is in front of us
  updateState();
  if(!engaged){
	//check if we are engaged
	
	if(crawlState == FRONT_BLOCKED){
	  if(disengaging){
	    network->angularRateSpeedCommand(-0.8,0.0);
	    disengage_count=0;
	    sleep(1);
	  }
	  else{
	  	//bind to the wall
	 	 engaged=true;
	  	//we need to draw a line in a mask, that marks the exit point
		 Point_<double> robot; 
		  robot.x=model->state.crioPosition.x*(1.0/model->meterToPixel)+model->translation.x;
  		robot.y=model->state.crioPosition.y*-(1.0/model->meterToPixel)+model->translation.y; 
  		cvCircle(goalPathMask,robot,.9/model->meterToPixel,cvScalar(0),-1,8);

 
		 network->sendWarning();
	  }
	}
	else{
	  if(disengage_count>10){
		disengaging=false;
	  }
	  else{
		disengage_count++;
	  }
	  //check if we are within some distance of our goal, if we are then pick a new goal
	  Point_<double> goalRelativeToBot;
	  goalRelativeToBot.x=goalPoint.x-model->state.crioPosition.x;
	  goalRelativeToBot.y=goalPoint.y-model->state.crioPosition.y;
	  if(goalLeeway> sqrt(goalRelativeToBot.x*goalRelativeToBot.x+goalRelativeToBot.y*goalRelativeToBot.y)){
		 network->sendWarning();
		 atGoal=true;
		 
	  }
	  else{
		//move to the goal
		//this may cause a problem with the shitty waypoint steering
		cout<<"sending goal point"<<endl;
		AngularRateHolder temp =simplesteering( goalPoint.x,goalPoint.y, model->state,.5,1.0,3/3.1415,0.9,3.1415/6);
		printf("sending vel command vel %f, omega %f\n", temp.vel,temp.omega);
		
		network->angularRateSpeedCommand( temp.omega,temp.vel);
		}
	}
  }
  else{
  
	if(checkOnPath() ){
	  //we have reconnected with the path
	  //find the heading that we need to turn to to face the goal and turn there
	  engaged=false;
	  disengaging=true;
	  
	  disengage_count=0;
	  return false;
	}
	
	//if we cannot disengage then just update according to the wall crawler
	updateCrawl();
  }
  return false;
}
