#include "GPSPlanner.h"

GPSPlanner::GPSPlanner(WorldModel* model, TrajectoryGenerator *trajgen,float goal_success_dist){
  //currently hard coding 150
  this->trajgen=trajgen;
  this->goal_success_dist=goal_success_dist;
  goalIndex=0;
  planImage=cvCloneImage(model->worldModel);
  cvSetZero(planImage);
  goalFinder=new GoalFinder(  cvPoint(model->worldModel->width,model->worldModel->height),150);
  this->model=model;
  this->network=network;
  astar=new AStar(cvPoint(model->worldModel->width,model->worldModel->height));
  
}

GPSPlanner::~GPSPlanner(){
  delete goalFinder;
  delete astar;
}

void GPSPlanner::setSingleGoal(CvPoint2D32f point){
  goals.clear();
  goals.push_back(point);
}

bool GPSPlanner::atGoal(){
  float diffx=state.crioPosition.x-goals[goalIndex].x;
  float diffy=state.crioPosition.y-goals[goalIndex].y;
  cout<<"at this goal index "<<goalIndex<<" dist "<<sqrt(diffx*diffx+diffy*diffy) <<endl;
  if(sqrt(diffx*diffx+diffy*diffy)<goal_success_dist){
	return true;
  }
  return false;
}

void GPSPlanner::loadGoals(string filepath){
  goals.clear();
  ifstream config_file(filepath.c_str());
  int number_of_points=0;
  CvPoint2D32f point;
  if (config_file.is_open())
  {
	config_file>>number_of_points;
	  for (int i=0;i<number_of_points;i++)
	  {
		  config_file>>point.x;
		  config_file>>point.y;
		  /*
latconv 111063.5799731039
longconv 81968

latoff 42.6778389
longoff-83.1952306


pos 1
13.6946 -103.267
0.405903 -101.794

 point.x=(point.x-42.6778389)*111063.5799731039;
 point.y=(point.y -(-83.1952306))*81968 ;
		  
		  */
		  float temp=(point.x-42.6778389)*111063.5799731039; 
		  point.x=(point.y -(-83.1952306))*81968 ;
		  point.y=temp;
		  
		 
		  
		  
		  goals.push_back(point);
		  cout<<"loaded point "<<point.x<<" "<<point.y<<endl;
	  }
	}
	else{
	  cout<<"failed to load gps points\n";
	}
	config_file.close();
}


bool GPSPlanner::updatePlanner(){
	state=model->state;
//  state=model->network->getCurrentPose();
  
  if(atGoal()){
	goalIndex++;
  }
  //if our index is outside the goal size then we are at the final point. no planning needed
  if(goalIndex<goals.size()){
	  
	CvPoint robotPos;
	robotPos.x=(int)(state.crioPosition.x*(1.0/model->meterToPixel))+model->translation.x;
	robotPos.y=(int)(state.crioPosition.y*(-1.0/model->meterToPixel))+model->translation.y;
	
	CvPoint goalpixelpos;
	goalpixelpos.x=(int)(goals[goalIndex ].x*(1.0/model->meterToPixel))+model->translation.x;
	goalpixelpos.y=(int)(goals[goalIndex ].y*(-1.0/model->meterToPixel))+model->translation.y;
	CvScalar checkclear;
	CvPoint nextBestGoal;
	bool goal_on_map=false;
	if(goalpixelpos.x>0 && goalpixelpos.y >0&&goalpixelpos.x<model->worldModel->width && goalpixelpos.y <model->worldModel->height){	
	  checkclear=cvGet2D(model->worldModel,goalpixelpos.y,goalpixelpos.x);
	  goal_on_map=true;
	}
	bool going_to_actual_point=false;
	if(goal_on_map && checkclear.val[0]<30||checkclear.val[0]>230){
	  cout<<"wee see the goal\n";
	  going_to_actual_point=true;
	  nextBestGoal=goalpixelpos;
	}
	else{
	  nextBestGoal = goalFinder->findBestPoint(model->frontierPixels,robotPos,state.crioRotRad,goalpixelpos);
	}
   cout<<"looking for goal "<< goalIndex<<" at "<<goals[goalIndex ].x<<" "<<goals[goalIndex ].y<<" goal pixel "<<goalpixelpos.x<<" "<<goalpixelpos.y<<endl;
	//cvCircle(planImage,nextBestGoal,2,cvScalar(255,255,255),-1,8);
	
	//this will only operate on lidar currently
	
	path=astar->planImage(model->worldModel,robotPos,nextBestGoal,state.crioPosition);
	
	cvSetZero(planImage);
	
	while(path.empty()){
	if(model->network!=NULL){
	 model->network->sendWarning();
	 }
		  //remove the explored goal from the frontier, if the goal is the actual goal, undiscover it
		  cout<<"impossible goal, need to remove "<<nextBestGoal.x<<" "<<nextBestGoal.y<<endl;
		  if(going_to_actual_point){
		  //remove the actual point from explored space
			cvSet2D(model->worldModel,nextBestGoal.y,nextBestGoal.x,cvScalar(model->neutral_knowledge,model->neutral_knowledge,model->neutral_knowledge)); 
		  }
		  else{
		  // remove the pixel from the frontier map 
			cvSet2D(model->frontierPixels,nextBestGoal.y,nextBestGoal.x,cvScalar(0,0,0)); 
		  }
		  nextBestGoal = goalFinder->findBestPoint(model->frontierPixels,robotPos,state.crioRotRad,goalpixelpos);
		path=astar->planImage(model->worldModel,robotPos,nextBestGoal,state.crioPosition);
	  }
	
	 
	  //draw the path and convert it to meters
	  for(int i=0;i<path.size();i++){
		cvCircle(planImage,cvPoint(path[i].x,path[i].y),2,cvScalar(255,255,255),-1,8);
  //	  cout<<"path "<<path[i].x<<" "<<path[i].y<<endl;
  //convert back to world coodinates
		path[i]=model->pixeltometer(path[i].x,path[i].y);
//		cout<<"path "<<path[i].x<<" "<<path[i].y<<endl;
	  }
	  this->trajgen->setPath(path);
	
  }
  else{
 	 return false;
  }
  return true;
}

