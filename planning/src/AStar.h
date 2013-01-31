/**
*	AStar.h
*	Author: Jesse Fish
*
* A* pather
*/


#ifndef _AStar_h
#define _AStar_h

#include <string>
#include "GoalFinder.h"
#include "WorldModel.h"
#include "Planner.h"
#include "WorldModel.h"
#include <vector>

using namespace std;


class AStarPoint{
  public:
	AStarPoint(){};
	AStarPoint(CvPoint pixel,int *cost)
	{
	  this->pixel=pixel;
	  this->cost=cost;
	  //cout<<"created point with cost "<<*cost<<endl;
	}
	CvPoint pixel;
	int *cost;
};

class AStarComparison
{
public:
  AStarComparison(){};
  bool operator() (const AStarPoint& lhs, const AStarPoint&rhs) const
  {
	//cout<<"cost1 "<<*(lhs.cost)<<" cost2 "<<*(rhs.cost)<<endl;
	 return (*(lhs.cost)> *(rhs.cost));
  }
};


class AStar{
	public:
		AStar(CvPoint imageSize);
		~AStar();
		vector<CvPoint2D32f> planImage(IplImage* codemap,CvPoint position, CvPoint goal, CvPoint2D32f realRobotPos,WorldModel *model=NULL,float maxOffset=0,long startTime=0,float * distance_off=NULL,bool* killBool=NULL);
		//remove this function after testing
		IplImage* closedList;
		IplImage* aStarCost;
		IplImage* optimisticCost;
		IplImage* optimisticCostBig;
		IplImage* totalCostMap;
		IplImage* parentxList;
		IplImage* parentyList;
		
		
	private:
	  void buildDistanceImage();
	  
	  AStarPoint evaluateNode(const AStarPoint topPoint, const int newx, const int newy, const int move);
	  int intstep;
	  int intchannels;
	  int bytestep;
	  int bytechannels;
	   IplImage* show;
};

void testAstar();
#endif

