/**
*	TrajectoryGenerator.h
*	Author: Jesse Fish
*
* TrajectoryGenerator
*/
#ifndef _TrajectoryGenerator_h
#define _TrajectoryGenerator_h

#include <vector>
#include "WorldModel.h"
#include "simplesteering.h"
#include "PositionState.h"



using namespace std;

class TrajectoryGenerator{
	public:
	  TrajectoryGenerator(float success_dist, bool post_process=false);
	  ~TrajectoryGenerator();
	  void setPath(vector <CvPoint2D32f> path);
	  bool updateRobot(WorldModel * model,int steps_ahead);
	  vector <CvPoint2D32f> path;
	  bool gotNewPath();
	  int pathIndex;

	private:
	  float success_dist;
	  bool post_process;
	  bool newpath;
	  pthread_mutex_t pathMut;
	  AngularRateHolder trajSteering(PositionState state );
};

#endif

