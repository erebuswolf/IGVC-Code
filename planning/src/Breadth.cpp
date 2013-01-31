#include "Breadth.h"
#include <queue>

Breadth::Breadth(){
	width  = 300;
	height = 300;
}

void Breadth::buildInitialMap(){
	cout<<"Started Breadth"<<endl;
	//grab the position of the robot
	CvPoint robot;
	robot.x = (int)(model->state.crioPosition.x*(1.0/model->meterToPixel)) + model->translation.x;
   	robot.y = (int)(model->state.crioPosition.y*-(1.0/model->meterToPixel)) + model->translation.y;
	cout<<"Grabbed robot position\n";
	//stores the current point to check
	CvPoint current, next;
	int values[4];
	
	//create the queue of points to check
	queue <CvPoint> points;
	
	//create the cost map
	model->costMap = cvCreateImage(cvGetSize(model->worldModel), IPL_DEPTH_32F, 1);
	cvSet(model->costMap, cvScalar(10000));
	cvSet2D(model->costMap,robot.x,robot.y,cvScalar(0));
	int floatstep = model->costMap->widthStep/sizeof(float);
  	int floatchannels = model->costMap->nChannels;
	cout<<"Created cost map\n";
	
	//store the pointer to the data
	int* costdata = (int *)model->costMap->imageData;
	
	//set the region of interest
	CvRect roi = cvRect(robot.x - width/2., robot.y - height/2., width, height);
	roi = model->correctROI(roi);
	cvSetImageROI(model->costMap, roi);
	
	//grab a copy of the combined image
	open = cvCreateImage(cvGetSize(model->worldModel), 8, 1);
	model->copyCompletePlanWorldmap(open);
	int intstep = open->widthStep/sizeof(int);
	int intchannels = open->nChannels/sizeof(int);
	int* opendata = (int*)open->imageData;
	
	//push the first point into the queue
	points.push(robot);
	
	cout<<"Initialize everything and starting loop\n";
	
	while(points.size()!=0){
		cout<<"Queue Size: "<< points.size()<<endl;
		waitKey(0);
		//read in the current front point
		current = points.front();
		
		//pop the point from the list
		points.pop();
		
		//check four neighbors for the lowest value
		values[0] = costdata[(current.y-1)*floatstep+(current.x)*floatchannels+0];
		values[1] = costdata[(current.y)*floatstep+(current.x-1)*floatchannels+0];
		values[2] = costdata[(current.y+1)*floatstep+(current.x)*floatchannels+0];
		values[3] = costdata[(current.y)*floatstep+(current.x+1)*floatchannels+0];
		
		for(int i=0; i<4; i++){
			//check to see if the neighbor is less than the current
			if(values[i]+1<costdata[(current.y)*floatstep+(current.x)*floatchannels+0]){
				costdata[(current.y)*floatstep+(current.x)*floatchannels+0] = values[i];
			}
			
			//check to see if neighbor should be added to list
			switch(i){
			case 0:
				next.x = current.x;
				next.y = current.y-1;
				break;
			case 1:
				next.x = current.x-1;
				next.y = current.y;
				break;
			case 2:
				next.x = current.x;
				next.y = current.y+1;
				break;
			case 3:
				next.x = current.x+1;
				next.y = current.y;
			}
			
			if(opendata[next.y*intstep+next.x+1*intchannels+0] < model->neutral_knowledge){
				//push value into queue
				points.push(next);
				
				//close the value on the image
				opendata[next.y*intstep+next.x+1*floatchannels+0] = 255;
			}
		}
	}
}

void Breadth::buildMap(){

}
