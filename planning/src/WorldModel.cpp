/**
*	WorldModel.cpp
*	Author: Jesse Fish
*
*/

#include "WorldModel.h"
#include "TimeKeeper.h"
#include "NetworkStack.h"
#include "packets.h"
#include <math.h>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

WorldModel::WorldModel(){
  
}

void WorldModel::init(string config_filepath){
  
  pthread_mutex_init(&(this->copyCompletePlanWorldmapMut),NULL);
  pthread_mutex_init(&(this->poseMut),NULL);
  pthread_mutex_init(&(this->TranlsationMut),NULL);
  pthread_mutex_init(&(this->copyfrontiermapMut),NULL);
  
  
  //read the config file in and read the angle offset
  //and translation offset
  ifstream config_file (config_filepath.c_str());
  planner_world=NULL;
  int width=0;
  int height=0;	
  
  int scanwidth=0;
  int scanheight=0;	
  
  state.crioPosition.x=0;
  state.crioPosition.y=0;
  state.crioRot=0;
  state.crioRotRad=0;
  first_shift=true;
  string log_filepath;
  
  if (config_file.is_open())
  {
	config_file>>useLidar;
	//	cout<<useLidar<<endl;
	config_file.ignore(300,'\n');
	config_file>>useVision;
	//		cout<<useVision<<endl;
	config_file.ignore(300,'\n');
	config_file>>usePresetMap;
	config_file.ignore(300,'\n');
	config_file>>usePresetMapPSO_ONLY;
	//	cout<<usePresetMap<<endl;
	config_file.ignore(300,'\n');
	config_file>>neutral_knowledge;
	config_file.ignore(300,'\n');
	config_file>>dintensityClear;
	config_file.ignore(300,'\n');
	config_file>>dintensityWall;
	config_file.ignore(300,'\n');
	config_file>>walldepth;
	config_file.ignore(300,'\n');
	config_file>>lidarThreshold;
	config_file.ignore(300,'\n');
	config_file>>objectDialation;
	config_file.ignore(300,'\n');
	config_file>>maxDeltaWall;
	config_file.ignore(300,'\n');
	config_file>>width;
	config_file.ignore(300,'\n');
	config_file>>height;
	config_file.ignore(300,'\n');
	config_file>>scanwidth;
	config_file.ignore(300,'\n');
	config_file>>scanheight;
	config_file.ignore(300,'\n');
	config_file>>meterToPixel;
	config_file.ignore(300,'\n');
	config_file>>translation.x;
	config_file.ignore(300,'\n');
	config_file>>translation.y;
	config_file.ignore(300,'\n');
	config_file>>mapMove;
	config_file.ignore(300,'\n');
	config_file>>post_process;
	config_file.ignore(300,'\n');
	config_file>>log_filepath;
	config_file.close();
  }
  else cerr << "Unable to open world model config file" << endl; 
  
  //allocate memory for the model image
  worldModel=cvCreateImage(cvSize(width,height),8,1);	
  worldModelBuffer=cvCreateImage(cvSize(width,height),8,1);	
  whereIHaveBeen=cvCreateImage(cvSize(width,height),8,1);
  frontierPixels=cvCreateImage(cvSize(width,height),8,1);
  
  if(useVision){
  	worldModelVision=cvCreateImage(cvSize(width,height),8,1);
  	worldModelVisionAdd=cvCreateImage(cvSize(width,height),8,1);
  	worldModelVisionSub=cvCreateImage(cvSize(width,height),8,1);
  	worldModelCombined=cvCreateImage(cvSize(width,height),8,1);
  }
  else{
  	worldModelVision=NULL;
  	worldModelCombined=NULL;
  }
  
  if(post_process)
  {
	cout<<"opening log "<<log_filepath<<endl;
  	logfile.open(log_filepath.c_str());
  }


//  last_danger=lidar->danger_radius;
last_danger=50;

  scanMask=cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  wallMask=cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  frontierMask=cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  
  completePlanWorldMap=cvCreateImage(cvSize(width,height),8,1);
  
  sonarAdd = cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  sonarSub = cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  sonarMap = cvCreateImage(cvSize(width,height),8,1);
  
  gradientAdd=cvCreateImage(cvSize(scanwidth,scanheight),8,1);
  
  cvSet(gradientAdd,cvScalar(dintensityClear));
  
  cvSet(worldModel,cvScalar(neutral_knowledge));
  cvSet(worldModelBuffer,cvScalar(neutral_knowledge));
  
  setupGradient(dintensityClear,-0.01);
  inDanger=false;
}


//fetches new information from the appropriate sources
int WorldModel::updateModel(){
  //collision = false;
  
  //	cvSet(worldModel,cvScalar(neutral_knowledge));
  //get position from crio anr orientation

///time stamp for reading logs
  long time_stamp=0;
  
  if(!usePresetMapPSO_ONLY){
  	//read pso from position from file
  	if(post_process)
  	{
  		if(logfile.is_open()&&!logfile.eof())
  		{
  			logfile >> time_stamp;
  			logfile.ignore(300,'\n');
  			logfile >> state.crioPosition.x;
  			logfile >> state.crioPosition.y;
  			logfile >> state.crioRotRad;
  			state.vel = 0;
  			logfile.ignore(300,'\n');
  		}
  		else
  		{
	  		logfile.close();
  			cout<<"Unable to read from log file\n";
  			return -1;
  		}
  	}
	else
	{
	  //lock the pose
		pthread_mutex_lock(&poseMut);
		Pose tstate=network->getCurrentPose();
		state.crioPosition.x=tstate.x;
		state.crioPosition.y=tstate.y;
		state.crioRotRad=tstate.theta;
		state.vel=tstate.vel;
		state.sonar=tstate.front_sonar_ping;
		pthread_mutex_unlock(&poseMut);
	}
  }
  else{
	//if using the lidar preset map pso only!!!
	state.crioPosition.y=state.estPosition.y;
	state.crioRotRad=state.estRotRad;
  }
  state.timeStamp=TimeKeeper::GetTime();
  
  if(first_shift){
	pthread_mutex_lock(&TranlsationMut);
	translation.x+=state.crioPosition.x*(1.0/meterToPixel);
	translation.y+=state.crioPosition.y*(-1.0/meterToPixel);
	cout<<"first shift "<<translation.x<<" "<<translation.y<<endl;
	first_shift=false;
	pthread_mutex_unlock(&TranlsationMut);
	
  }
  
  cout<<"position data "<<state.crioPosition.x<<" "<<state.crioPosition.y<<" "<<state.crioRotRad<<" vel "<<state.sonar<<endl;
  
  //this polar to deg conversion should be a compile constant
  state.crioRot=state.crioRotRad;//*180/(3.1415);
  
  //network code here to listen for crio data
  //gather all data first
  if(useLidar){
	//	cout<<"using lidar"<<endl;
	lidar->getScan();
	//do the fast converstion before anything else so we can send reflexive halt
	int danger=0;
	if(walldepth>0){
	  danger=lidar->convertLast(state.crioRot,walldepth);
	}
	else{
	  danger=lidar->convertLast(state.crioRot,walldepth,lidarThreshold);
	  //danger=false;
	}
	if(danger<last_danger-5 || state.sonar<0.5){
	//|| state.sonar<0.5
	  //send reflexivestop
	  cout<<"sonar "<<state.sonar<<endl;
	  printf("warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	//TODO: make this not a constant
	  if(state.vel>.4){
		network->sendWarning();
		inDanger=true;
	  }
//	  collision = true;//who made this variabe?
	}
	
	  if(last_danger<danger){
		  last_danger++;
		  last_danger=min(last_danger,50);
	  }
	  else{
	  last_danger=danger;
	  }
  }
  
  if(useVision){
	//cout<<"using vision"<<endl;
	if(post_process){
	  cout<<"loading image with stamp "<<time_stamp<<endl;
	  vision->manager->updateCameras(time_stamp);
	}else{
	   vision->manager->updateCameras(); 
	}
  }
  //calc position from lidar map if we have preknown map.
  //do this before calling updateModelLidar! that call manipulates the lidar data
  if(usePresetMap){
	findPositionFromMap();
  }
  
  //add lidar to model
  //	lidar->lidarData.time_stamp
  if(useLidar){
	if(walldepth>0)
	//  updateModelLidar();
		cout<<"bad config file, using old lidar method"<<endl;
	else
	  updateModelLidarWithDialation();
  }
  
  if(useVision){
	updateModelVision();
  }
  
  //update the sonar
  updateSonar();
  
  buildCompletePlanWorldmap();
  
  
  return 0;
  //add code here to draw where we have been in a seperate map
  //	cvCircle(whereIHaveBeen,);
}
void WorldModel::buildCompletePlanWorldmap(){
//  cout<<"Building Complete World Map"<<endl;
  
  cvCopy(worldModel,completePlanWorldMap);
  //cout<<"Copied image"<<endl;
  cvAdd(sonarMap,completePlanWorldMap,completePlanWorldMap);
  
  //TODO:draw 4 lines here to be the points that bound the obsticle course
  //40 pixels=2 meters thick
  float pointx=-29.2205;
  float pointy=123.2;
  CvPoint point1;
  point1.x=pointx*(1.0/meterToPixel)+translation.x;
  point1.y=pointy*(-1.0/meterToPixel)+translation.y;

  pointx=-29.2205;
  pointy=188.2;
  CvPoint point2;
  point2.x=pointx*(1.0/meterToPixel)+translation.x;
  point2.y=pointy*(-1.0/meterToPixel)+translation.y;
  
  
  pointx=25.7795;
  pointy=188.2;
  CvPoint point3;
  point3.x=pointx*(1.0/meterToPixel)+translation.x;
  point3.y=pointy*(-1.0/meterToPixel)+translation.y;

  pointx=25.7795;
  pointy=123.2;
  CvPoint point4;
  point4.x=pointx*(1.0/meterToPixel)+translation.x;
  point4.y=pointy*(-1.0/meterToPixel)+translation.y;
//	cout<<"points 1"<<point1.x<<" "<<point1.y<<endl;
// 	cout<<"points 2"<<point2.x<<" "<<point2.y<<endl;
// 	cout<<"points 3"<<point3.x<<" "<<point3.y<<endl;
// 	cout<<"points 4"<<point4.x<<" "<<point4.y<<endl;
 
  cvLine(completePlanWorldMap, point1, point2, cvScalar(255), 40);
  cvLine(completePlanWorldMap, point2, point3, cvScalar(255), 40);
  cvLine(completePlanWorldMap, point3, point4, cvScalar(255), 40);
  cvLine(completePlanWorldMap, point1, point4, cvScalar(255), 40);
  //cvShowImage("wee",completePlanWorldMap);
  
  //cout<<"Added image"<<endl;
  if(worldModelVisionAdd!=NULL){
  //	cout<<"Using vision"<<endl;

	cvAdd(worldModelVisionAdd,completePlanWorldMap,completePlanWorldMap);

//	cout<<"Added vision"<<endl;
  }
}

//update the model off of the lidar data, this will change the lidar data, so make sure to call the preset map functions before this one if we are using them.
/*void WorldModel::updateModelLidar(){
  
  //would be nice to have this make a copy of the lidar data instead of using it directly
  
  //set the masks to zero
  cvSetZero(scanMask);
  cvSetZero(wallMask);
  
  //scale the x and y points to be in the scale we are dealing with
  cvScale(lidar->lidarData.x,lidar->lidarData.x,1.0/meterToPixel);
  cvScale(lidar->lidarData.y,lidar->lidarData.y,1.0/-meterToPixel);
  
  cvScale(lidar->lidarData.xb,lidar->lidarData.xb,1.0/meterToPixel);
  cvScale(lidar->lidarData.yb,lidar->lidarData.yb,1.0/-meterToPixel);
  
  //transform the points from their original position to be centered in the scan mask
  cvAddS(lidar->lidarData.x,cvScalar(scanMask->width/2),lidar->lidarData.x);
  cvAddS(lidar->lidarData.y,cvScalar(scanMask->height/2),lidar->lidarData.y);
  cvAddS(lidar->lidarData.xb,cvScalar(scanMask->width/2),lidar->lidarData.xb);
  cvAddS(lidar->lidarData.yb,cvScalar(scanMask->height/2),lidar->lidarData.yb);
  
  //deal with clear space
  //draw a curve that is from the center of the scan mask to all of the points in the inner scan
//  cout<<lidar->lidarData.numPoints<<endl;
  for(int i=0;i<lidar->lidarData.numPoints;i++){
	
	curve1[i].x=(int)(cvmGet(lidar->lidarData.x,i,0));
	curve1[i].y=(int)(cvmGet(lidar->lidarData.y,i,0));
	//	cout<<curve1[i].x<<" "<<curve1[i].y<<endl;
  }
  curve1[lidar->lidarData.numPoints].x=scanMask->width/2;
  curve1[lidar->lidarData.numPoints].y=scanMask->height/2;
  CvPoint* curveArr[1]={curve1};
  int      nCurvePts[1]={lidar->lidarData.numPoints};
  int      nCurves=1;
  cvFillPoly(scanMask,curveArr,nCurvePts,nCurves,cvScalar(255,255,255));
  
  //deal with walls
  //draw a mask that is from the outside of the boundary mask
  //current index in the curve we are in
  int curve_index=0;
  //last index in the matrix we accessed
  int last_index=0;
  int current_beginning=0;
  for(int i=0;i<=lidar->lidarData.numPoints;i++){
	if(abs((int)(cvmGet(lidar->magb,i,0))-(int)(cvmGet(lidar->magb,last_index,0)))<maxDeltaWall &&i<lidar->lidarData.numPoints){
	  curve2[curve_index].x=(int)(cvmGet(lidar->lidarData.xb,i,0));
	  curve2[curve_index].y=(int)(cvmGet(lidar->lidarData.yb,i,0));
	  last_index=i;
	  curve_index++;
	}
	
	else{
	  for(int j=i-1;j>=current_beginning;j--){
		curve2[curve_index]=curve1[j];
		curve_index++;
	  }
	  
	  //draw the curve here
	  CvPoint* curveArr2[1]={curve2};
	  int      nCurvePts2[]={curve_index};
	  int      nCurves2=1;
	  cvFillPoly(wallMask,curveArr2,nCurvePts2,nCurves2,cvScalar(255,255,255));
	  if(i<lidar->lidarData.numPoints){
		curve_index=0;
		current_beginning=i;
		last_index=i;
		//move i back 1 so we dont drop a point
		i--;
	  }
	}
  }
  
  
  CvRect roi;
  //the remapping from the lidar space to the robot space happens in lidar manager
  //make the x and y be the translation
  roi.x=(int)(state.crioPosition.x*(1.0/meterToPixel))+translation.x-scanMask->width/2;
  roi.y=(int)(state.crioPosition.y*-(1.0/meterToPixel))+translation.y-scanMask->height/2;
  
  roi.width=scanMask->width;
  roi.height=scanMask->height;
  
  //	cout<<roi.x<<" "<<roi.y<<" "<<roi.width<<" "<<roi.height<<endl
  
  //Call to move map if necessary
  roi = correctROI(roi);
  
  //cout<<"left loop\n";
  //cout<<roi.x<<" "<<roi.y<<" "<<roi.width<<" "<<roi.height<<endl;
  
  cvSetImageROI(worldModel,roi);
  cvSub(worldModel,gradientAdd,worldModel,scanMask);
  cvAdd(worldModel,gradientAdd,worldModel,wallMask);
  cvResetImageROI(worldModel);
  
  //command to draw a circle on the position the robot is in
  
}*/

void WorldModel::updateModelLidarWithDialation(){
 //cout<<"starting lidar stuff"<<endl;
 // 
//  objectDialation
  
  //set the masks to zero
  cvSetZero(scanMask);
  cvSetZero(wallMask);
  cvSetZero(frontierMask);
//  frontierPixels
  
  //scale the x and y points to be in the scale we are dealing with
  cvScale(lidar->lidarData.x,lidar->lidarData.x,1.0/meterToPixel);
  cvScale(lidar->lidarData.y,lidar->lidarData.y,1.0/-meterToPixel);
  
  cvScale(lidar->lidarData.xb,lidar->lidarData.xb,1.0/meterToPixel);
  cvScale(lidar->lidarData.yb,lidar->lidarData.yb,1.0/-meterToPixel);
  
  //transform the points from their original position to be centered in the scan mask
  cvAddS(lidar->lidarData.x,cvScalar(scanMask->width/2),lidar->lidarData.x);
  cvAddS(lidar->lidarData.y,cvScalar(scanMask->height/2),lidar->lidarData.y);
  cvAddS(lidar->lidarData.xb,cvScalar(scanMask->width/2),lidar->lidarData.xb);
  cvAddS(lidar->lidarData.yb,cvScalar(scanMask->height/2),lidar->lidarData.yb);
  
  
  //deal with clear space
  //draw a curve that is from the center of the scan mask to all of the points in the inner scan
  //cout<<lidar->lidarData.numPoints<<endl;
  for(int i=0;i<lidar->lidarData.numPoints;i++){
	
	curve1[i].x=(int)(cvmGet(lidar->lidarData.x,i,0));
	curve1[i].y=(int)(cvmGet(lidar->lidarData.y,i,0));
//	cout<<"i "<<i<<endl;
  }
// cout<<curve1[181].x<<", "<<curve1[181].y<<" P";

//  cout<<endl;
  curve1[lidar->lidarData.numPoints].x=scanMask->width/2;
  curve1[lidar->lidarData.numPoints].y=scanMask->height/2;
  CvPoint* curveArr[1]={curve1};
  int      nCurvePts[1]={lidar->lidarData.numPoints};
  int      nCurves=1;
  cvFillPoly(scanMask,curveArr,nCurvePts,nCurves,cvScalar(255));
  
  //deal with the frontier pixels
  cvPolyLine(frontierMask,curveArr,nCurvePts,nCurves,1,cvScalar(255),1);

  //deal with walls
  
  //draw a mask that is from the outside of the boundary mask
  //current index in the curve we are in
  
//  cout<<(cvmGet(lidar->mag,0,0))<<endl;
  for(int i=0;i<=lidar->lidarData.numPoints-2;i++){
	if( !(lidar->lidarData.thresholdedPoints[i]) && !(lidar->lidarData.thresholdedPoints[i+1]) &&abs((int)(cvmGet(lidar->mag,i,0))-(int)(cvmGet(lidar->mag,i+1,0)))<maxDeltaWall ){
//	cout<<"i "<<i<<endl;
	  cvLine(wallMask, curve1[i], curve1[i+1], cvScalar(255),objectDialation/meterToPixel);
	  
	  // 
	}
  }
  
  CvRect roi;
  //the remapping from the lidar space to the robot space happens in lidar manager
  //make the x and y be the translation
  roi.x=(int)(state.crioPosition.x*(1.0/meterToPixel))+translation.x-scanMask->width/2;
  roi.y=(int)(state.crioPosition.y*-(1.0/meterToPixel))+translation.y-scanMask->height/2;
  
  roi.width=scanMask->width;
  roi.height=scanMask->height;
  
  //Call to move map if necessary
  roi = correctROI(roi);
  
 // cout<<"left loop\n";
  cvSub(scanMask,wallMask,scanMask);
  
  cvSub(scanMask,frontierMask,scanMask);
  cvSub(frontierMask,wallMask,frontierMask);
  
  cvCircle(frontierMask,cvPoint(scanMask->width/2,scanMask->height/2),0.4/meterToPixel,cvScalar(255,255,255),1,8); 
  
  cvSub(frontierMask,scanMask,frontierMask);
  
  cvCircle(scanMask,cvPoint(scanMask->width/2,scanMask->height/2),0.4/meterToPixel,cvScalar(255,255,255),-1,8);
  
  //do this early
  cvSetImageROI(worldModel,roi);
  
  
  
  pthread_mutex_lock(& copyfrontiermapMut );
  cvSetImageROI(frontierPixels,roi);
  cvAdd(frontierMask,frontierPixels,frontierPixels);
  
  //remove any frontier pixels that are discovered as free space
  //threshold below neutral knowledge and make those values bright, then subtract them from the frontier
  //just use frontierMask as a temp area
  //TODO: make 20 a config value rather than constant
  
  cvSub(worldModel,gradientAdd,worldModel,scanMask);
  //cvShowImage("scan mask",scanMask);
  cvAdd(worldModel,gradientAdd,worldModel,wallMask);
 // cvShowImage("wall mask",wallMask);
  
  cvThreshold(worldModel,frontierMask,neutral_knowledge-2,255,CV_THRESH_BINARY_INV);
  cvSub(frontierPixels,frontierMask,frontierPixels);
  
  //remove any frontier pixels that are discovered as a wall
  cvThreshold(worldModel,frontierMask,neutral_knowledge+2,255,CV_THRESH_BINARY);
  cvSub(frontierPixels,frontierMask,frontierPixels);
  
 // cvShowImage("scan",scanMask);
//  cvShowImage("wall",wallMask);
  
  cvResetImageROI(worldModel);
  
  //cvShowImage( "frontier",frontierPixels);
  cvResetImageROI(frontierPixels);
  pthread_mutex_unlock(& copyfrontiermapMut );
// cout<<"ending lidar"<<endl; 
}

CvRect WorldModel::getRegionCentered( int width,int height){
  CvRect roi;
  roi.x=(int)(state.crioPosition.x*(1.0/meterToPixel))+translation.x-width/2;
  roi.y=(int)(state.crioPosition.y*-(1.0/meterToPixel))+translation.y-height/2;
  
  roi.width=width;
  roi.height=height;
  return roi;
}

///if we are walking off the map, this shifts the vision and lidar maps and all other maps
/// such that we are no longer walking off the edge, then it returns the correct roi for
/// the new maps
CvRect WorldModel::correctROI(CvRect roi){
//printf("correcing ROI\n");
  //initialize to nonvalid values they will be set in the loop
  int changed=0;
  int xtrans=1;
  int ytrans=1;
  //we need to stay in this loop untill the rio is valid
  while(xtrans!=0||ytrans!=0){
//	cout<<"correcting RIO!!!\n"<<state.crioPosition.x<<" "<<state.crioPosition.y<<endl;
	//check if the rio is valid, if it is not valid then we need to move in either 1 or 2 directions
	if(roi.x<0){
	  xtrans=-1;
	//  printf("less than 0 x\n");
	}
	else if(roi.x+roi.width>worldModel->width){
	  xtrans=1;
	//  printf("greater than 0 x\n");
	}
	else{
	  xtrans=0;
	 // printf("fine x\n");
	}
	
	if(roi.y<0){
	  ytrans=-1;
	//  printf("less than 0 y\n");
	}
	else if(roi.y+roi.height>worldModel->height){
	  ytrans=1;
	//  printf("greater than 0 y\n");
	}
	else{
	  ytrans=0;
	//  printf("fine y\n");
	}
	//cout<<"trans "<<xtrans<<" "<<ytrans<<endl;
	//cout<<"trans x,y"<<translation.x<<" "<<translation.y<<endl;	
	if(xtrans!=0||ytrans!=0){
	  changed=1;
	  //lock the mutex
	  pthread_mutex_lock(&TranlsationMut);
	  translation.x += -mapMove*xtrans;
	  translation.y += -mapMove*ytrans;
	  pthread_mutex_unlock(&TranlsationMut);
	  
	  //cout<<"new trans x,y"<<translation.x<<" "<<translation.y<<endl;
	  //change the roi values
	  roi.x = (int)(state.crioPosition.x*(1.0/meterToPixel))+translation.x-roi.width/2;
	  roi.y = (int)(state.crioPosition.y*-(1.0/meterToPixel))+translation.y-roi.height/2;
	  
//	  printf("recalculated ROI %d %d\n",roi.x,roi.y);
	  CvRect roiImage;
	  CvRect roiImageBuffer;
	  
	  roiImage.x = max(mapMove*xtrans,0);
	  roiImageBuffer.x = max(-mapMove*xtrans,0);
	  if(xtrans!=0){
		roiImage.width = worldModel->width-mapMove;
		roiImageBuffer.width = worldModel->width-mapMove;
	  }
	  else{
		roiImage.width = worldModel->width;
		roiImageBuffer.width=worldModel->width;
	  }
	  
	  roiImage.y = max(mapMove*ytrans,0);
	  roiImageBuffer.y = max(-mapMove*ytrans,0);
	  if(ytrans!=0){
		roiImage.height = worldModel->height-mapMove;
		roiImageBuffer.height = worldModel->height-mapMove;
	  }
	  else{
		roiImage.height = worldModel->height;
		roiImageBuffer.height = worldModel->height;
	  }
	  
	  //	cout<<"made regions of interest\n";
	  //move the image the ammount we want and swap the buffers
	  //set the buffer to all grey, this could/should be optomized to set only the region we aren't drawing in to grey
	  
	  fixROI(worldModel, worldModelBuffer,roiImage,roiImageBuffer,neutral_knowledge);
	  IplImage* temp=worldModel;
  	  worldModel=worldModelBuffer;
  	  worldModelBuffer=temp;
  	  
  	  //fix sonar map
  	  fixROI(sonarMap, worldModelBuffer, roiImage, roiImageBuffer, 0);
  	  temp = sonarMap;
  	  sonarMap = worldModelBuffer;
  	  worldModelBuffer = temp;
  	  
	  if(useVision){
		  //do it again for the vision image
		fixROI(worldModelVision, worldModelBuffer,roiImage,roiImageBuffer,0);
		temp=worldModelVision;
  		worldModelVision=worldModelBuffer;
  		worldModelBuffer=temp;
  		//do it for vision add
  		fixROI(worldModelVisionAdd, worldModelBuffer, roiImage, roiImageBuffer, 0);
  		temp = worldModelVisionAdd;
  		worldModelVisionAdd = worldModelBuffer;
  		worldModelBuffer = temp;
  		//do it for vision sub
  		fixROI(worldModelVisionSub, worldModelBuffer, roiImage, roiImageBuffer, 0);
  		temp = worldModelVisionSub;
  		worldModelVisionSub = worldModelBuffer;
  		worldModelBuffer = temp;
	}
	  
	  pthread_mutex_lock(& copyfrontiermapMut );
	  fixROI(frontierPixels, worldModelBuffer,roiImage,roiImageBuffer,0);
	  temp=frontierPixels;
  	  frontierPixels=worldModelBuffer;
  	  worldModelBuffer=temp;
	  pthread_mutex_unlock(& copyfrontiermapMut);
  	  
	
	  if(planner_world!=NULL){
		  fixROI(planner_world, worldModelBuffer,roiImage,roiImageBuffer,0);
		  cvCopy(temp,planner_world);
	  }
		if(costMap!=NULL){
			fixROI(costMap, worldModelBuffer,roiImage,roiImageBuffer,0);
			temp = costMap;
			costMap = worldModelBuffer;
			worldModelBuffer = temp;
		}
	}
  }
  return roi;
}

void WorldModel::fixROI(IplImage* image, IplImage* buffer,CvRect roiImage,CvRect roiImageBuffer,int value){	  
  cvSet(buffer,cvScalar(value));
  cvSetImageROI(image,roiImage);
  cvSetImageROI(buffer,roiImageBuffer);
  cvCopy(image,buffer);
  cvResetImageROI(image);
  cvResetImageROI(buffer);
}


//update the model off of the Vision data
void WorldModel::updateModelVision(){
  vision->findLinesAndObsticles(state.crioRotRad);
  
  //vision->processedImage;
  
  
  CvRect roi;
  //the remapping from the lidar space to the robot space happens in lidar manager
  //make the x and y be the translation
  roi.x=(int)(state.crioPosition.x*(1.0/meterToPixel))+translation.x-vision->processedImage->width/2;
  roi.y=(int)(state.crioPosition.y*-(1.0/meterToPixel))+translation.y-vision->processedImage->height/2;
  
  roi.width=vision->processedImage->width;
  roi.height=vision->processedImage->height;
  
  //inverse the scan mask
  cvThreshold( scanMask, scanMask, 128, 255, CV_THRESH_BINARY_INV);
  
  //remove objects and space behind objects for adder map
  cvSetImageROI(vision->processedImage, cvRect( (vision->processedImage->width - scanMask->width)/2.,
  												(vision->processedImage->height - scanMask->height)/2.,
  												scanMask->width, scanMask->height) );				
  cvSub(vision->processedImage, scanMask, vision->processedImage);
//  cvShowImage("overlay", vision->processedImage);
 // cvWaitKey(30);
  cvResetImageROI(vision->processedImage);
  
  //Call to move map if necessary
  roi = correctROI(roi);
  cvSetImageROI(worldModelVision,roi);
  
  //cvCopy(vision->processedImage,worldModelVision,vision->processedImage);
  cvAdd(worldModelVision,vision->processedImage,worldModelVision);
  cvSub(worldModelVision,vision->subtractorImage,worldModelVision);
  
  cvResetImageROI(worldModelVision);
  //take the vision output image and add it to our vision
  //model with the appropriate rotation and position
  
  cvOr(worldModel,worldModelVision,worldModelCombined);
  
}

//assume a 10 degree sonar
void WorldModel::updateSonar(){
	CvPoint end;

//	cout<<state.sonar<<" sonarval"<<endl;
	//find the radius of the circle
	float distance = state.sonar*(1.0/meterToPixel);
	double radius  = state.sonar*atan(10*3.14159/180.)*(1.0/meterToPixel);
	
	//find the unit vector of the heading
	CvPoint2D32f u_vec = cvPoint2D32f(-sin(state.crioRotRad), -cos(state.crioRotRad));
	CvPoint2D32f n_vec = cvPoint2D32f(sin(state.crioRotRad + (3.14159/2.)) ,cos(state.crioRotRad+ (3.14159/2.))); 
	//draw a circle on the add mask
	cvSetZero(sonarAdd);
			end.x = (sonarSub->width/2.) + distance*u_vec.x;
		end.y = (sonarSub->height/2.) + distance*u_vec.y;

	if(state.sonar<2){
			cvCircle(sonarAdd, end, radius, cvScalar(dintensityClear), -1);
	}	//cvShowImage("circle", sonarAdd);
	
	//draw triangle on subtract mask
	cvSetZero(sonarSub);
	CvPoint points[3];
	CvPoint* triangle[1] = {points};
	points[0] = cvPoint( sonarAdd->width/2.,sonarAdd->height/2.0);
	points[1] = cvPoint( end.x - n_vec.x*radius, end.y - n_vec.y*radius);
	points[2] = cvPoint( end.x + n_vec.x*radius, end.y + n_vec.y*radius);
	int nPts[1];
	nPts[0] = 3;
	cvFillPoly(sonarSub,triangle,nPts,1,cvScalar(dintensityClear-20));
	//cvShowImage("triangle", sonarSub);
	
	//set region of interest
	CvRect roi=getRegionCentered(sonarSub->width,sonarSub->height);
	
	//correct the roi if necessary
	roi = correctROI(roi);
	
	//add and subtract maps into sonar map
	cvSetImageROI(sonarMap, roi);
	cvSub(sonarMap, sonarSub, sonarMap);
	cvAdd(sonarMap, sonarAdd, sonarMap);
	cvShowImage("Sonar", sonarMap);
	//cvWaitKey(0);
	cvResetImageROI(sonarMap);
}

void WorldModel::informHardware(){
  if(usePresetMap){
	//send corrected position information back to hardware
//	network->sendTweak(float(state.estPosition.x-state.crioPosition.x),float(state.estPosition.y-state.crioPosition.y),float(state.estRotRad-state.crioRotRad));
 //this existed in morgans network code, but does not exist in ben chad and erics
  }
}

void WorldModel::findPositionFromMap(){
  // Calculate the position from a known map, position, and heading
  lidarLocalizer->estimatePosition(&state,lidar->lidarData.scanPoints,lidar->lidarData.numPoints);
  informHardware();
}

void WorldModel::setupGradient(int max, float slope){
  CvPoint center;
  center.x=gradientAdd->width/2;
  center.y=gradientAdd->height/2;
  //will start by making a simple linear gradient out from the center;
  for(int i=sqrt(gradientAdd->width*gradientAdd->width+gradientAdd->height*gradientAdd->height);i>1;i--){
	cvCircle(gradientAdd,center,i,cvScalar(max+i*slope),-1,8);
  }
}


CvPoint2D32f WorldModel::pixeltometer(int x, int y){
  CvPoint2D32f value;
  value.x=((x)-translation.x)*meterToPixel;
  //meter to pixel needs to be reversed here on the y side so up is up
  value.y=((y)-translation.y)*-meterToPixel;
  return value;
}

WorldModel::~WorldModel(){
  
  cvReleaseImage(&worldModel);
  cvReleaseImage(&scanMask);
  cvReleaseImage(&gradientAdd);
  cvReleaseImage(&whereIHaveBeen);
  
}

//threadsafe way to get the translation values
CvPoint2D32f WorldModel::getTranslation(){
  CvPoint2D32f value;
  pthread_mutex_lock(&TranlsationMut);
  value=translation;
  pthread_mutex_unlock(&TranlsationMut);
  return value;
}

//threadsafe way to get the state of the robot
PositionState WorldModel::getstate(){
  pthread_mutex_lock(&poseMut);
  PositionState value= state;
  pthread_mutex_unlock(&poseMut);
  return value;
}
	
void WorldModel::copyCompletePlanWorldmap(IplImage * image){
  pthread_mutex_lock(& copyCompletePlanWorldmapMut );
  cvCopy( completePlanWorldMap,image);
  pthread_mutex_unlock(& copyCompletePlanWorldmapMut);
}

void WorldModel::copyFrontiermap(IplImage * image){
  pthread_mutex_lock(& copyfrontiermapMut );
  cvCopy( frontierPixels,image);
  pthread_mutex_unlock(& copyfrontiermapMut);
}
