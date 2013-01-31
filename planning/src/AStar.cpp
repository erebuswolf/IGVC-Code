#include "AStar.h"
#include <stdio.h>
#include <math.h>
#include "TimeKeeper.h"
#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include "PositionState.h"
using namespace std;



AStar::AStar(CvPoint imageSize){
  
  aStarCost=cvCreateImage(cvSize(imageSize.x,imageSize.y),IPL_DEPTH_32S,1);
  totalCostMap=cvCreateImage(cvSize(imageSize.x,imageSize.y),IPL_DEPTH_32S,1);
  optimisticCost=cvCreateImage(cvSize(imageSize.x,imageSize.y),IPL_DEPTH_32S,1);
  optimisticCostBig=cvCreateImage(cvSize(imageSize.x*2,imageSize.y*2),IPL_DEPTH_32S,1);
  closedList=cvCreateImage(cvSize(imageSize.x,imageSize.y),8,1);
  parentxList=cvCreateImage(cvSize(imageSize.x,imageSize.y), IPL_DEPTH_32S,1);
  parentyList=cvCreateImage(cvSize(imageSize.x,imageSize.y), IPL_DEPTH_32S,1);
  
  show=cvCloneImage(totalCostMap);
  
  
  intstep = aStarCost->widthStep/sizeof(int);
  intchannels = aStarCost->nChannels;
  
  bytestep = closedList->widthStep/sizeof(uchar);
  bytechannels = closedList->nChannels;
  buildDistanceImage();
}

AStar::~AStar(){
  cvReleaseImage(&totalCostMap);
  cvReleaseImage(&aStarCost);
  cvReleaseImage(&optimisticCost);
  cvReleaseImage(&optimisticCostBig);
  cvReleaseImage(&closedList);
  cvReleaseImage(&parentxList);
  cvReleaseImage(&parentyList);
}

//plan from goal to our position
vector<CvPoint2D32f> AStar::planImage(IplImage* codemap,CvPoint robotPosition, CvPoint goal,CvPoint2D32f realRobotPos,WorldModel *model,float maxOffset,long startTime,float* distance_off,bool *killBool){
  cout<<"starting path planning"<<endl;
  int iterations=0;
  
  vector<CvPoint2D32f> path_to_goal;
  //set the roi of the optimistic cost map and copy it to total cost map
  cvSet(aStarCost,cvScalar(-1,-1,-1));
  cvSet(parentxList,cvScalar(-1,-1,-1));
  cvSet(parentyList,cvScalar(-1,-1,-1));
  cvSet(totalCostMap,cvScalar(-1,-1,-1));
  cvSet(closedList,cvScalar(0,0,0));
  
  CvRect roi;
  roi.width=codemap->width;
  roi.height=codemap->height;
  roi.x=roi.width-robotPosition.x;
  roi.y=roi.height-robotPosition.y;
  
  if(roi.x<0||roi.y<0||roi.x>roi.width||roi.y>roi.height){
	cout<<"why is your position outside the image!!!!?\n";
  }
  cvSetImageROI(optimisticCostBig,roi);
  cvCopy(optimisticCostBig,optimisticCost);
  cvResetImageROI(optimisticCostBig);
  
  CvPoint currentPosition=goal;
  
  //begin astar
  int * stepdata = (int *)aStarCost->imageData;
  int * totalcostdata = (int *)totalCostMap->imageData;
  int * optdata = (int *)optimisticCost->imageData;
  
  int * parentxdata = (int *) parentxList->imageData;
  int * parentydata = (int *) parentyList->imageData;
  
  uchar * closedListdata = (uchar *)closedList->imageData;
  uchar * modeldata = (uchar *)codemap->imageData;
  
//ASTAR QUEUE
  vector <AStarPoint> explorePoints;
  
  AStarPoint pointExpanding;
  pointExpanding.cost=&(totalcostdata[currentPosition.y*intstep+currentPosition.x*intchannels+0]);
  pointExpanding.pixel.x=currentPosition.x;
  pointExpanding.pixel.y=currentPosition.y;
 //set the first value to be its own parent 
  parentxdata[currentPosition.y*intstep+currentPosition.x*intchannels+0]=currentPosition.x;
  parentydata[currentPosition.y*intstep+currentPosition.x*intchannels+0]=currentPosition.y;
  
  explorePoints.push_back(pointExpanding);
  
  //start at 1 step to ensure we can remove this mask when we need to from another image
  stepdata[currentPosition.y*intstep+currentPosition.x*intchannels+0]=1;
  totalcostdata[currentPosition.y*intstep+currentPosition.x*intchannels+0]=optdata[currentPosition.y*intstep+currentPosition.x*intchannels+0];
  
//  cout<<"initial cost "<<*(pointExpanding.cost)<<endl;
  
  const float square_mov=10;
  const float diag_mov=14.5;
  
  bool foundGoal=false;
  
  AStarPoint topPoint;
  bool first_point=true;
  long resettime=startTime;
  while(!explorePoints.empty() && (killBool==NULL || !(*killBool))){
//	cout<<"expingblah\n"<<endl;
	 
//	cout<<"exping\n"<<(killBool==NULL)<<endl;
	  
// cout<<"exping\n"<<*killBool<<endl;
	  
	usleep(50);
	iterations++;
	if(model!=NULL){
	  resettime=TimeKeeper::GetTime();
	  PositionState pos=model->getstate();
	  float diffx=pos.crioPosition.x-realRobotPos.x;
	  float diffy=pos.crioPosition.y-realRobotPos.y;
	  *distance_off=sqrt(diffx*diffx+diffy*diffy);
	  if((*distance_off)>maxOffset){
		return path_to_goal;
	  }
	}
	
	topPoint=explorePoints.back();
	explorePoints.pop_back();
	closedListdata[topPoint.pixel.y*bytestep+topPoint.pixel.x*bytechannels+0]=1;
	
	/*if(iterations>1000)
	{
		//fuck it and leave
		foundGoal=true;
		break;
	}*/
	
	//cout<<(int)(modeldata[topPoint.pixel.y*bytestep+topPoint.pixel.x*bytechannels+0])<<" pixel value"<<endl;
	  
	//if the space is clear
	//cout<<"first pixel"<<topPoint.pixel.x<<" "<<topPoint.pixel.y<<" "<<*(topPoint.cost)<<endl;
	if(topPoint.pixel.x==robotPosition.x&&topPoint.pixel.y==robotPosition.y){
		foundGoal=true;
		break;
	  }
	  //TODO: 75 as a config variable
	if(modeldata[topPoint.pixel.y*bytestep+topPoint.pixel.x*bytechannels+0]<75||first_point){
	  first_point=false;
	  int newx;
	  int newy;
	  //check for edges of screen
	  bool clear_above=false;
	  bool clear_below=false;
	  bool clear_left=false;
	  bool clear_right=false;
	  
	  
	  if(topPoint.pixel.y>0)
		clear_above=true;
	  if(topPoint.pixel.y<codemap->height)
		clear_below=true;
	  
	  if(topPoint.pixel.x>0)
		clear_left=true;
	  if(topPoint.pixel.x<codemap->width)
		clear_right=true;
	  
	  //check if point has been added to the open list
	  //if it hasn't or if we can improve its cost update it and change the parent
	  
	  
	  //UP DOWN LEFT RIGHT
	  if(clear_above){
		//add point directly above
		newx=topPoint.pixel.x;
		newy=topPoint.pixel.y-1;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, square_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	  
	  if(clear_below){
		//add point directly below
		newx=topPoint.pixel.x;
		newy=topPoint.pixel.y+1;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, square_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	  
	  if(clear_left){
		//add point directly left
		newx=topPoint.pixel.x-1;
		newy=topPoint.pixel.y;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, square_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	  if(clear_right){
		//add point directly right
		newx=topPoint.pixel.x+1;
		newy=topPoint.pixel.y;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, square_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
		//DIAGONALS
	  
	  if(clear_left&&clear_above){
		//add point directly above and left
		newx=topPoint.pixel.x-1;
		newy=topPoint.pixel.y-1;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, diag_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		
		}
	  }
	  if(clear_right&&clear_above){
		//add point directly above and left
		newx=topPoint.pixel.x+1;
		newy=topPoint.pixel.y-1;		
		AStarPoint temp=evaluateNode(topPoint, newx, newy, diag_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	  if(clear_left&&clear_below){
		//add point directly above and left
		newx=topPoint.pixel.x-1;
		newy=topPoint.pixel.y+1;		
		AStarPoint temp=evaluateNode(topPoint, newx, newy, diag_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	  if(clear_right&&clear_below){
		//add point directly above and left
		newx=topPoint.pixel.x+1;
		newy=topPoint.pixel.y+1;
		AStarPoint temp=evaluateNode(topPoint, newx, newy, diag_mov);
		if(temp.cost!=NULL){
		  explorePoints.push_back(temp);
		}
	  }
	}
//	cout<<"sortin\n";

	sort(explorePoints.begin(),explorePoints.end(),AStarComparison());
	/*double immin=0;
	double immax=0;
	CvPoint minloc;
	CvPoint maxloc;
	cvCopy(totalCostMap,show);
    cvMinMaxLoc(show,&immin,&immax,&minloc,&maxloc);
    cvConvertScale(show,show,2147483647/immax);
	cvShowImage("total cost",show);	
	cvWaitKey(3);
	*/
  }
  
  if(foundGoal){
	//cout<<"found goal!"<<endl;
	CvPoint2D32f lastparent;
	lastparent.x=(float)topPoint.pixel.x;
	lastparent.y=(float)topPoint.pixel.y;
	//cout<<"x "<<lastparent.x<<" y "<<lastparent.y<<endl;
	path_to_goal.push_back(lastparent);
	
	while(1){
	  CvPoint2D32f parent;
	  parent.x=(float)(parentxdata[int(lastparent.y)*intstep+int(lastparent.x)*intchannels+0]);
	  parent.y=(float)(parentydata[int(lastparent.y)*intstep+int(lastparent.x)*intchannels+0]);
	  if(lastparent.x!=parent.x || lastparent.y!=parent.y){
	//	cout<<"goal pushed\n";
		lastparent=parent;
		path_to_goal.push_back(parent);
	  }
	  else{
		break;
	  }
	}
  }
  
 // cout<<"ending\n";
  return path_to_goal;
}
AStarPoint AStar::evaluateNode(const AStarPoint topPoint, const int newx, const int newy, const int move){
  //add point directly above and left
	if( ((uchar *)closedList->imageData)[newy*bytestep+newx*bytechannels+0]==0){
	  bool addtoqueue=false;
	  if(((int *)aStarCost->imageData)[newy*intstep+newx*intchannels+0]<0 || ((int *)aStarCost->imageData)[newy*intstep+newx*intchannels+0]> ((int *)aStarCost->imageData)[topPoint.pixel.y*intstep+topPoint.pixel.x*intchannels+0]+move){
		((int *)aStarCost->imageData)[newy*intstep+newx*intchannels+0] = ((int *)aStarCost->imageData)[topPoint.pixel.y*intstep+topPoint.pixel.x*intchannels+0]+move;
		((int *)totalCostMap->imageData)[newy*intstep+newx*intchannels+0]=((int *)aStarCost->imageData)[newy*intstep+newx*intchannels+0]+((int *)optimisticCost->imageData)[newy*intstep+newx*intchannels+0];
		
		((int *)parentxList->imageData)[newy*intstep+newx*intchannels+0]=topPoint.pixel.x;
		((int *)parentyList->imageData)[newy*intstep+newx*intchannels+0]=topPoint.pixel.y;
		addtoqueue=true;
		//cout<<((int *)parentxList->imageData)[newy*intstep+newx*intchannels+0]<<" "<<((int *)parentyList->imageData)[newy*intstep+newx*intchannels+0];
	  }
	  if(addtoqueue){
		return(AStarPoint(cvPoint(newx,newy),&(((int *)totalCostMap->imageData)[newy*intstep+newx*intchannels+0])));
	  }
	}
  	return(AStarPoint(cvPoint(newx,newy),NULL));  
}
void AStar::buildDistanceImage(){
  CvPoint  center;
  center.x=optimisticCostBig->width/2;
  center.y=optimisticCostBig->height/2;
  
  int step = optimisticCostBig->widthStep/sizeof(int);
  int channels = optimisticCostBig->nChannels;
  int * data = (int *)optimisticCostBig->imageData;
  CvPoint unitCurPoint;
  for(int x=0;x<optimisticCostBig->width;x++){
	for(int y=0;y<optimisticCostBig->height;y++){
	  unitCurPoint.x=abs(x-center.x);
	  unitCurPoint.y=abs(y-center.y);
	  data[y*step+x*channels+0] = 10*(unitCurPoint.x+unitCurPoint.y)+1;
	 // cout<<data[y*step+x*channels+0]<<endl;
	//  sqrt(unitCurPoint.x*unitCurPoint.x+unitCurPoint.y*unitCurPoint.y);
	  //
	  
	}
  }
}

void testAstar(){
  AStar search(cvPoint(500,500));
  
  TimeKeeper::start_time();
  long start=0;
  IplImage* test=cvCreateImage(cvSize(500,500),8,1);
  cvCircle(test,cvPoint(70,100),30,cvScalar(255,255,255),-1,8);
  cvCircle(test,cvPoint(150,130),30,cvScalar(255,255,255),-1,8);
  
  vector<CvPoint2D32f> path;
  for(int i=0;i<100;i++){
  start=TimeKeeper::GetTime();
	cout<<"starting\n";
	path=search.planImage(test,cvPoint(50,50),cvPoint(200,200),cvPoint2D32f(0,0));
	cout<<"found in "<<TimeKeeper::GetTime()-start<<" microseconds"<<endl;
  }
  /*for(int i=0;i<path.size();i++){
	cout<<path[i].x<<" "<<path[i].y<<endl;
  }*/
  cvReleaseImage(&test);
}
