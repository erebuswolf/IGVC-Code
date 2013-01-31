#include "GoalFinder.h"
#include <stdio.h>
#include <math.h>
#include "TimeKeeper.h"

#include <iostream>
using namespace std;

///image size is the size of world model image, heading rad is the number of pixels that we care about for the heading of the robot
GoalFinder::GoalFinder(CvPoint imageSize, int heading_rad){
  costFrontiers=cvCreateImage(cvSize(imageSize.x,imageSize.y),IPL_DEPTH_32F,1);
  
  costMapDistance=cvCreateImage(cvSize(imageSize.x*2,imageSize.y*2),IPL_DEPTH_32F,1);
  costMapHeading=cvCreateImage(cvSize(heading_rad*2,heading_rad*2),IPL_DEPTH_32F,1);
  
  costRotMapHeading=cvCloneImage(costMapHeading);
  
  buildDistanceMask();
  //build it at pi/2 to make our coordinates the stupid ones we use with 0 heading pointing up the y axis
  buildHeadingMask(3.1415/2);
}

GoalFinder::~GoalFinder(){
  cvReleaseImage(&costMapDistance);
  cvReleaseImage(&costMapHeading);
  cvReleaseImage(&costRotMapHeading);
  cvReleaseImage(&costFrontiers);
}
//robot position in pixel coordinates
CvPoint GoalFinder::findBestPoint(IplImage* frontier_map,CvPoint robotPosition,float robotHeading,CvPoint goalPosition){
  //rotate the heading image to have the robots heading
  float rot_mat_arr[2][3];
  CvMat rot_mat=cvMat(2,3,CV_32F,rot_mat_arr);
  cout<<robotHeading<<endl;
  
  CvPoint2D32f center=cvPoint2D32f(costMapHeading->width/2.f , costMapHeading->height/2.f);
  //rotation is expecting deg
  cv2DRotationMatrix(center,robotHeading*180/3.1415,1,&rot_mat);
  cvWarpAffine(costMapHeading,costRotMapHeading,&rot_mat,CV_INTER_LINEAR|CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
  
  CvRect roi;
  roi.width=costRotMapHeading->width;
  roi.height=costRotMapHeading->height;
  roi.x=robotPosition.x-roi.width/2;
  roi.y=robotPosition.y-roi.height/2;
  
  if(roi.x<0||roi.y<0||roi.x+roi.width>costFrontiers->width||roi.y+roi.height>costFrontiers->height){
	cout<<"WARNING ROBOT IS OFF THE MAP, CANNOT PLAN TO IT"<<endl;
  }
  
  cvSetZero(costFrontiers);
  cvSetImageROI(costFrontiers,roi);
  cvAdd(costFrontiers,costRotMapHeading,costFrontiers);
  cvResetImageROI(costFrontiers);
  
  roi.width=frontier_map->width;
  roi.height=frontier_map->height;
  
  roi.x=roi.width-robotPosition.x;
  roi.y=roi.height-robotPosition.y;
  
  
  cvSetImageROI(costMapDistance,roi);
  cvAdd(costFrontiers,costMapDistance,costFrontiers);
  cvResetImageROI(costMapDistance);
  
  
  
  //change to the goal position
  roi.x=roi.width-goalPosition.x;
  roi.y=roi.height-goalPosition.y;
  if(roi.x<0||roi.y<0||roi.x>roi.width||roi.y>roi.height){
	//cout<<"WARNING GPS GOAL IS OFF THE MAP, CANNOT PLAN TO IT"<<endl;
	//cout<<"roi is "<<roi.x<<" "<<roi.y<<" "<<roi.width<<" "<<roi.height<<endl;
	
	
	CvPoint2D32f vec2goal;
	vec2goal.x=goalPosition.x-robotPosition.x;
	vec2goal.y=goalPosition.y-robotPosition.y;
	float vec2goalmag=sqrt(vec2goal.x*vec2goal.x+vec2goal.y*vec2goal.y);
	
	//very stupid threshold right now
	
	
	if(roi.x<0){
	  roi.x=0;
	}
	else if(roi.x>=roi.width){
	  roi.x=roi.width;
	}
	if(roi.y<0){
	  roi.y=0;}
	else if(roi.y>=roi.height){
	  roi.y=roi.height;}
	
//	cout<<"roi is "<<roi.x<<" "<<roi.y<<" "<<roi.width<<" "<<roi.height<<endl;
  }
  
  
  cvSetImageROI(costMapDistance,roi);
  cvAdd(costFrontiers,costMapDistance,costFrontiers);
  cvResetImageROI(costMapDistance);
  
  
  double immin=0;
  double immax=0;
  CvPoint minloc;
  CvPoint maxloc;
  
  cvMinMaxLoc(costFrontiers,&immin,&immax,&minloc,&maxloc,frontier_map);
  cvConvertScale(costFrontiers,costFrontiers,1.0/immax);
  //  cvShowImage("image2",costFrontiers);
 // cout<<"min "<<minloc.x<<" "<<minloc.y<<endl;
  return minloc;
}


void GoalFinder::removePoint(IplImage* frontier_map,CvPoint point){
  cvSet2D(frontier_map,point.y,point.x,cvScalar(0));
}

void GoalFinder::buildDistanceMask(){
  IplImage* centerDot=cvCreateImage(cvSize(costMapDistance->width,costMapDistance->height),IPL_DEPTH_8U,1);
  CvPoint center;
  center.x=costMapDistance->width/2;
  center.y=costMapDistance->height/2;
  cvSet(centerDot,cvScalar(1));
  cvCircle(centerDot,center,1,cvScalar(0.0),-1,8);
  cvDistTransform(centerDot,costMapDistance,CV_DIST_L2);
  //CV_DIST_MASK_PRECISE
  cvReleaseImage(&centerDot);

}

void GoalFinder::buildHeadingMask(float heading){
  //pick the point, determine its distance from the center, determing its angle from the center, and headingMask
  CvPoint2D32f center;
  center.x=costMapHeading->width/2;
  center.y=costMapHeading->height/2;
  float val=0;
  
  //x,y
  CvPoint2D32f headingVec=cvPoint2D32f(0,1);
  
  headingVec.x=cos(heading);
  headingVec.y=-sin(heading);
  CvPoint2D32f unitCurPoint;
  float dist=0;
  
  float angle=0;
  
  int step = costMapHeading->widthStep/sizeof(float);
  int channels = costMapHeading->nChannels;
  float * data = (float *)costMapHeading->imageData;
  
  float too_close=5;

  float max_dist=costMapHeading->width/2;
//  sqrt(headingMask->width/2*headingMask->width/2+headingMask->height/2*headingMask->height/2);
  for(int x=0;x<costMapHeading->width;x++){
	for(int y=0;y<costMapHeading->height;y++){
	  unitCurPoint.x=x-center.x;
	  unitCurPoint.y=y-center.y;
	  dist=sqrt(unitCurPoint.x*unitCurPoint.x+unitCurPoint.y*unitCurPoint.y);
	  if(dist>too_close){
		unitCurPoint.x/=dist;
		unitCurPoint.y/=dist;
		angle=acos(headingVec.x*unitCurPoint.x+headingVec.y*unitCurPoint.y);
		val=angle*max(-dist+max_dist,0.f)/float(max_dist*.008);
		data[y*step+x*channels+0] = val;
	  }
	  else{
		data[y*step+x*channels+0] = angle+3.1415*(25)*max(max_dist,500.f)/float(max_dist);
	  }
	}
//	printf("%f val\n",val);
  }
  
}



void testGoalFinding(){
  
  //made up variables
  CvPoint position=cvPoint(700,300);
  IplImage * frontier_images=cvCreateImage(cvSize(1500,1500),IPL_DEPTH_8U,1);
  IplImage * goal=cvCreateImage(cvSize(1500,1500),IPL_DEPTH_8U,1);
  cvCircle(frontier_images, position, 70, cvScalar(255,255,255), 1);
  
  TimeKeeper::start_time();
  int size=1500;
  //create the goal finder
  printf("building maps\n");
  
  
  ///need this!!!
  GoalFinder gf(cvPoint(size,size),150);
  printf("maps done\n");
  
  cvNamedWindow("image2", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("image", CV_WINDOW_AUTOSIZE );
  int key=-1;
  
  double immin=0;
  double immax=0;
  CvPoint minloc;
  CvPoint maxloc;
  
  CvPoint goalLoc;
  
  float headingvalue=0;
  long start=0;
  
  while((key&255 )!= 32){
	cvSetZero(goal);
	start=TimeKeeper::GetTime();
//	headingvalue+=.3;
	
  ///need this!!!
	goalLoc=gf.findBestPoint(frontier_images,position,headingvalue,cvPoint(300,300));
	
	cvSet2D(goal,goalLoc.y,goalLoc.x,cvScalar(255));
	
	cvMinMaxLoc(gf.costFrontiers,&immin,&immax,&minloc,&maxloc);
	cvConvertScale(gf.costFrontiers,gf.costFrontiers,1.0/immax);
	
	cout<<TimeKeeper::GetTime()-start<<endl;
	cvShowImage("image2",gf.costFrontiers);
	cvShowImage("image",gf.costRotMapHeading);
	
	key=cvWaitKey(5);
  }
  
}
