/**
*	ThreadGPSPlanner.h
*	Author: Jesse Fish
*
* ThreadGPSPlanner
*/


#ifndef _ThreadGPSPlanner_h
#define _ThreadGPSPlanner_h

#include <string>
#include "GoalFinder.h"
#include "WorldModel.h"
#include "Planner.h"
#include "AStar.h"
#include "TrajectoryGenerator.h"

using namespace std;

class ThreadGPSPlanner: public Planner{
	public:
		ThreadGPSPlanner(WorldModel* model, TrajectoryGenerator *trajgen,float goal_success_dist);
		~ThreadGPSPlanner();
		
		bool updatePlanner();
		float goal_success_dist;
		GoalFinder * goalFinder;
		WorldModel* model;
		bool changinggoal;
		Network* network;
		vector<CvPoint2D32f> goals;
		int goalIndex;
		vector <CvPoint2D32f> path;
		bool killSearch;
		
		bool atGoal();
		void loadGoals(string filepath);
		void setSingleGoal(CvPoint2D32f point);
		
		CvPoint2D32f robotAtPathSend;
		bool sentFirst;
		bool killthread;
		
		void startThread();
		
	private:
	  pthread_t updateThread;
	  int thread_id;
	  pthread_mutex_t goalMut;
	  
	  AStar *astar;
	  TrajectoryGenerator *trajgen;
	  PositionState state;
	  IplImage * SearchImage;
	  IplImage* frontierCopy;
	  static void *startThread(void* voidPlanner);
};

#endif
