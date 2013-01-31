/**
*	GPSPlanner.h
*	Author: Jesse Fish
*
* bug planner
*/


#ifndef _GPSPlanner_h
#define _GPSPlanner_h

#include <string>
#include "GoalFinder.h"
#include "WorldModel.h"
#include "Planner.h"
#include "AStar.h"
#include "TrajectoryGenerator.h"

using namespace std;

class GPSPlanner: public Planner{
	public:
		GPSPlanner(WorldModel* model, TrajectoryGenerator *trajgen,float goal_success_dist);
		~GPSPlanner();
		bool updatePlanner();
		//void addGPSPoint(CvPoint2D32f newgoal);
		//remove this function after testing
		//void setGPSPoint(CvPoint2D32f newgoal);
		float goal_success_dist;
		GoalFinder * goalFinder;
		WorldModel* model;
		Network* network;
		vector<CvPoint2D32f> goals;
		int goalIndex;
		vector <CvPoint2D32f> path;
		bool atGoal();
		void loadGoals(string filepath);
		void setSingleGoal(CvPoint2D32f point);
	private:
	  AStar *astar;
	  TrajectoryGenerator *trajgen;
	  PositionState state;
};

#endif

