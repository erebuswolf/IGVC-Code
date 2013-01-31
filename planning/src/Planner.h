/**
*	Planner.h
*	Author: Jesse Fish
*
* looks at the model and plans where to go next
*
* This should be an abstract fucking class that real planners inherit from!!!!!!!
*/


#ifndef _Planner_h
#define _Planner_h

#include <string>
#include "WorldModel.h"


using namespace std;

class Planner{
	public:
		WorldModel* model;
		Network* network;
		//output image for debugging
		IplImage* planImage;
		virtual bool updatePlanner()=0;
	private:
	//	virtual void planPath()=0;
	//	virtual void sendPath()=0;
};

#endif

