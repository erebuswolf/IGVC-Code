/**
*	PointAndClickDisplay.cpp
*	Author: Jesse Fish
*
*/

#include "PointAndClickDisplay.h"
#include <stdio.h>
#include "simplesteering.h"

PointAndClickDisplay* pThis;


PointAndClickDisplay::PointAndClickDisplay(WorldModel* model):Display(model){
	cvSetMouseCallback(this->windowname.c_str(),&(PointAndClickDisplay::mouse_handler),this);
	pThis = this;
}

PointAndClickDisplay::~PointAndClickDisplay(){

}

void PointAndClickDisplay::mouse_handler(int event, int x, int y, int flags, void *param){
	if(event==CV_EVENT_LBUTTONDOWN){
		pThis->goalWayPoint=((PointAndClickDisplay*)(param))->model->pixeltometer(x+((PointAndClickDisplay*)(param))->windata.xmin,y+((PointAndClickDisplay*)(param))->windata.ymin);
		
		AngularRateHolder temp =simplesteering( pThis->goalWayPoint.x,pThis->goalWayPoint.y , ((PointAndClickDisplay*)(param))->model->state);
		
		printf("sending waypoint to position x %f, y %f\n", pThis->goalWayPoint.x, pThis->goalWayPoint.y);
//send point
		printf("sending vel command vel %f, omega %f\n", temp.vel,temp.omega);
		((PointAndClickDisplay*)(param))->model->network->angularRateSpeedCommand(temp.vel, temp.omega);
	}
	
}
