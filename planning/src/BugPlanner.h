/**
*	BugPlanner.h
*	Author: Jesse Fish
*
* bug planner
*/


#ifndef _BugPlanner_h
#define _BugPlanner_h

#include <string>
#include "WallCrawler.h"
#include "WorldModel.h"


using namespace std;

class BugPlanner: public WallCrawler{
	public:
		BugPlanner(double left_wall_dist, double left_wall_leeway,WorldModel* model, Network* network);
		~BugPlanner();
		bool updatePlanner();
		void setGoal(double x, double y);
		bool atGoal;
		
		Point_<double> goalPoint;
	private:
		void drawGoalPath();
		IplImage* goalPathMask;
		IplImage* robotMask;
		IplImage* debugmask;
		bool engaged;
		bool disengaging;
		bool checkOnPath();
		int disengage_count;
		double goalLeeway;
		
};

#endif

