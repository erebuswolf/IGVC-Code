/**
*	WallCrawler.cpp
*	Author: Jesse Fish
*
* wall crawling planner
*
*/

#include "WallCrawler.h"
#include <math.h>
WallCrawler::WallCrawler(double left_wall_dist, double left_wall_leeway,WorldModel* model, Network* network){
modeltouse=cvCloneImage(model->completePlanWorldMap);
  
  this->model=model;
  this->network=network;
  this->left_wall_dist=left_wall_dist;
  left_mask_length=1.4*left_wall_dist;
  
  crawlState = ON_WALL;
  
  model->meterToPixel;
  maskWidth=int((1.0/model->meterToPixel)*(left_wall_dist+(left_wall_leeway))*4);
  
  //cornering_Const=1.7;
  robot_length=.5;
  
 // carrot_length=1.5;
  
  maskHeight=maskWidth;
  leftMask=cvCreateImage(cvSize(maskWidth,maskHeight),8,1);
  cvSetZero(leftMask);
  
  leftCloseMask=cvCloneImage(leftMask);
  leftCenterMask=cvCloneImage(leftMask);
  forwardMask=cvCloneImage(leftMask);
  leftForwardMask=cvCloneImage(leftMask);
  mathMask=cvCloneImage(leftMask);
 // allMasks=cvCloneImage(leftMask);
  
  center=cvPoint(maskWidth/2,maskHeight/2);
}

WallCrawler::~WallCrawler(){
  cvReleaseImage(&leftMask);
  cvReleaseImage(&forwardMask);
  cvReleaseImage(&leftCenterMask);
  cvReleaseImage(&leftCloseMask);
  cvReleaseImage(&leftForwardMask);
 // cvReleaseImage(&allMasks);
  cvReleaseImage(&mathMask);
  
}

void WallCrawler::buildMasks(){
  
  cvSetZero(leftMask);
  cvSetZero(forwardMask);
  cvSetZero(leftForwardMask);
  //cvSetZero(allMasks);
  
  //robot is 1 meter
  double forward_clearspace=.05;
  
  double forward_offset=.1/(model->meterToPixel);
  
  Point_<double> leftVector;
  leftVector.x=-1;
  leftVector.y=0;
  
  Point_<double> forwardVector;
  forwardVector.x=0;
  forwardVector.y=1;
  
  leftMaskTranslation.x=0;
  leftMaskTranslation.y=.1/(model->meterToPixel);
  forwardMaskTranslation.x=0;
  forwardMaskTranslation.y=.7/(model->meterToPixel);
  leftforwardMaskTranslation.x=0;
  leftforwardMaskTranslation.y=.7/(model->meterToPixel);
  
  //correct for heading rotation
  double theta=(model->state.crioRotRad);
  
  Point_<double> temp_point;
  temp_point=leftVector;
  leftVector.x=temp_point.x *cos(theta)-temp_point.y*sin(theta);
  leftVector.y=temp_point.x *sin(theta)+temp_point.y*cos(theta);
  
		Point_<double> goalPoint;
  temp_point=forwardVector;
  forwardVector.x=temp_point.x *cos(theta)-temp_point.y*sin(theta);
  forwardVector.y=temp_point.x *sin(theta)+temp_point.y*cos(theta);
  
  
  
  temp_point=leftMaskTranslation;
  leftMaskTranslation.x=temp_point.x *cos(theta)-temp_point.y*sin(theta);
  leftMaskTranslation.y=temp_point.x *sin(theta)+temp_point.y*cos(theta);
  
  
  temp_point=forwardMaskTranslation;
  forwardMaskTranslation.x=temp_point.x *cos(theta)-temp_point.y*sin(theta);
  forwardMaskTranslation.y=temp_point.x *sin(theta)+temp_point.y*cos(theta);
  
  temp_point=leftforwardMaskTranslation;
  leftforwardMaskTranslation.x=temp_point.x *cos(theta)-temp_point.y*sin(theta);
  leftforwardMaskTranslation.y=temp_point.x *sin(theta)+temp_point.y*cos(theta);
  
  
  // cout<<leftforwardMaskTranslation.y<<" "<<leftforwardMaskTranslation.x<<" "<<sin(theta)<<endl;
  
  //draw left mask
  //add translation to A and B for the heading offset
  Point_<double> A;
  A.x=center.x+leftMaskTranslation.x;
  A.y=center.y-leftMaskTranslation.y;
  Point_<double> B;
  B.x=center.x+leftMaskTranslation.x+ leftVector.x*(left_mask_length)/model->meterToPixel;
  B.y=center.y-leftMaskTranslation.y-leftVector.y*(left_mask_length)/(model->meterToPixel);
  
  cvLine(leftMask,A,B,cvScalar(255),robot_length/model->meterToPixel);
  
  //draw left center mask
  B.x=center.x+leftMaskTranslation.x+ leftVector.x*(left_wall_dist)/model->meterToPixel;
  B.y=center.y-leftMaskTranslation.y-leftVector.y*(left_wall_dist)/(model->meterToPixel);
  cvLine(leftCenterMask,A,B,cvScalar(255),robot_length/model->meterToPixel);

  //draw left close mask 
  // there is kind of a random .9 constant here, consider revising
  B.x=center.x+leftMaskTranslation.x+ leftVector.x*(left_wall_dist*.75)/model->meterToPixel;
  B.y=center.y-leftMaskTranslation.y-leftVector.y*(left_wall_dist*.75)/(model->meterToPixel);
  cvLine(leftCloseMask,A,B,cvScalar(255),robot_length/model->meterToPixel);

 
  
//draw forward mask  
  A.x=center.x+forwardMaskTranslation.x;
  A.y=center.y-forwardMaskTranslation.y;
  B.x=center.x+forwardMaskTranslation.x+ forwardVector.x*(forward_clearspace)/model->meterToPixel;
  B.y=center.y-forwardMaskTranslation.y-forwardVector.y*forward_clearspace/(model->meterToPixel);
  
  //add translation to A and B for the heading offset
  cvLine(forwardMask,A,B,cvScalar(255),1.2*robot_length/model->meterToPixel);
  
  
  A.x=center.x+leftforwardMaskTranslation.x;
  A.y=center.y-leftforwardMaskTranslation.y;
  B.x=center.x+leftforwardMaskTranslation.x+ leftVector.x*(left_mask_length)/model->meterToPixel;
  B.y=center.y-leftforwardMaskTranslation.y-leftVector.y*(left_mask_length)/(model->meterToPixel);
  
  //add translation to A and B for the heading offset
  cvLine(leftForwardMask,A,B,cvScalar(255),robot_length/model->meterToPixel);
  
  //cvAdd(leftForwardMask,forwardMask,allMasks);
 // cvAdd(allMasks,leftMask,allMasks);
//  planImage=allMasks;
}

bool WallCrawler::updatePlanner(){
  model->updateModel();
  
  updateState();
  
  updateCrawl();
  return false;
}

void WallCrawler::updateCrawl(){
  if(crawlState == FRONT_BLOCKED){
		cout<<" forward wall, go right\n";
	network->angularRateSpeedCommand(-0.7,0.0);
  }
  else if(crawlState == CORNER){
	cout<<"corner wall, go left around it\n";
	
	//cout<<"sending angular rates"<<endl;
	network->angularRateSpeedCommand(0.65, 0.65);
	
  }
  else{
	//we have a wall on our left or we have no wall at all, just go, adjust as necissary
	float headchange=findDistToWall()*(0.6);
	cout<<"wall on left, stay the course\n"<<headchange<< " move with line"<<endl;

	network->angularRateSpeedCommand(headchange, .9);

  }
  
}
void WallCrawler::updateState(){
  buildMasks();
  
  //using the model find a decent way to crawl along a wall 
  CvRect roi=model->getRegionCentered(maskWidth,maskHeight);
  
	cvCopy(model->completePlanWorldMap,modeltouse);
  
/*  if(model->worldModelCombined!=NULL){
  	modeltouse=model->worldModel;
  	//modeltouse=model->worldModelCombined;
  }else{
  	modeltouse=model->worldModel;
 
   }*/
//  modeltouse=model->completePlanWorldMap;
 //	modeltouse=model->worldModel;
 
  cvSetImageROI(modeltouse,roi);
  // cout<<roi.width<<" "<<roi.height<<" "<<maskWidth<<endl;
  
  cvAnd(modeltouse,leftMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar leftSumsca=cvSum(mathMask);
  
  cvAnd(modeltouse,forwardMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar forwardSum=cvSum(mathMask);
  
  cvAnd(modeltouse,leftForwardMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar forwardLeftSum=cvSum(mathMask);
  
  
  
  cvResetImageROI(modeltouse);
 // cout<<forwardSum.val[0]<<" "<<leftSumsca.val[0]<<" "<<forwardLeftSum.val[0]<<endl;
  //check if there is wall in front. if so turn right
  if(forwardSum.val[0]>0){
	crawlState = FRONT_BLOCKED;	
  }else if(leftSumsca.val[0]>0&&forwardLeftSum.val[0]==0 ||leftSumsca.val[0]==0&&forwardLeftSum.val[0]==0){
	//else check if there is a wall to our left but we have reached a corner
	crawlState = CORNER;
  }
  else{
	crawlState = ON_WALL;
  }
}

//returns a unit vector point_double that represents a vector along the direction of the wall on our left
float WallCrawler::findDistToWall(){
 
//    IplImage* modeltouse=NULL;
  
  if(model->worldModelCombined!=NULL){
  	modeltouse=model->worldModel;
  	//modeltouse=model->worldModelCombined;
  }else{
  	modeltouse=model->worldModel;
  }
  
  CvRect roi=model->getRegionCentered(maskWidth,maskHeight);
  
  cvSetImageROI(modeltouse,roi);
  
  cvAnd(modeltouse,leftCenterMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar leftcenterSumsca=cvSum(mathMask);
  
  cvAnd(modeltouse,leftCloseMask,mathMask);
  cvThreshold(mathMask, mathMask, model->neutral_knowledge+10,255,CV_THRESH_BINARY);
  CvScalar leftCloseSumsca=cvSum(mathMask);
  
  cvResetImageROI(modeltouse);
  
  double move_const=.2;
  float hoz_dist_to_move=0;
  
  
  if(leftcenterSumsca.val[0]==0){
  	//we should move more left
  	hoz_dist_to_move+=move_const;
  }
  else if(leftCloseSumsca.val[0]>0){
  	//we should move more right
  	hoz_dist_to_move-=move_const;
  }
  
  // hoz_dist_to_move is the difference from how far we want to be from the wall, to how far we are
  cout<<"hoz move "<<hoz_dist_to_move<<endl;
 
  return  hoz_dist_to_move;
}
