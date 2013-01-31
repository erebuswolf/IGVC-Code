#include "ThreadGPSPlanner.h"
#include "TimeKeeper.h"

ThreadGPSPlanner::ThreadGPSPlanner(WorldModel* model, TrajectoryGenerator *trajgen,float goal_success_dist){
  //currently hard coding 150
  this->trajgen=trajgen;
  this->goal_success_dist=goal_success_dist;
  goalIndex=0;
  planImage=cvCloneImage(model->worldModel);
  SearchImage=cvCloneImage(model->worldModel);
  frontierCopy=cvCloneImage(model->worldModel);
  cvSetZero(planImage);
  goalFinder=new GoalFinder(  cvPoint(model->worldModel->width,model->worldModel->height),150);
  this->model=model;
  this->network=network;
  astar=new AStar(cvPoint(model->worldModel->width,model->worldModel->height));
  killthread=true;
  sentFirst=false;
  pthread_mutex_init(&(this->goalMut),NULL);
  killSearch=false;
}

ThreadGPSPlanner::~ThreadGPSPlanner(){
  killthread=true;
  pthread_join( updateThread,NULL);
  delete goalFinder;
  delete astar;
}

void ThreadGPSPlanner::setSingleGoal(CvPoint2D32f point){
  killSearch=true;
  pthread_mutex_lock(& goalMut );
  goals.clear();
  goals.push_back(point);
  pthread_mutex_unlock(& goalMut );
  killSearch=false;
}


void ThreadGPSPlanner::startThread(){
  if(killthread){
	killthread=false;
	thread_id=pthread_create(&updateThread,NULL,&startThread,(void*)this);
  }
  else{
	printf("Error: planning thread has already been started\n");
  }
}

bool ThreadGPSPlanner::atGoal(){
  PositionState tempstate=model->getstate();
  float diffx=tempstate.crioPosition.x-goals[goalIndex].x;
  float diffy=tempstate.crioPosition.y-goals[goalIndex].y;
//  cout<<"at this goal index "<<goalIndex<<" dist "<<sqrt(diffx*diffx+diffy*diffy) <<endl;
  if(sqrt(diffx*diffx+diffy*diffy)<goal_success_dist){
	return true;
  }
  return false;
}

void ThreadGPSPlanner::loadGoals(string filepath){
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

bool ThreadGPSPlanner::updatePlanner(){
  while(!killthread){
	pthread_mutex_lock(& goalMut );
	
//	cout<<"planning\n"<<endl;
	//long startTime=TimeKeeper::GetTime();
	
	state=model->getstate();
	
	//threadsafe way to get the translation values
	CvPoint2D32f translation=model->getTranslation();
	//	cout<<"copy maps"<<endl;

	model-> copyCompletePlanWorldmap(SearchImage);
	model-> copyFrontiermap(frontierCopy);
	//	cout<<"fail"<<endl;
	
	//SearchImage
	
	if(atGoal()){
	  goalIndex++;
	}
	//if our index is outside the goal size then we are at the final point. no planning needed
	//cout<<"not yet"<<endl;
	if(goalIndex<goals.size()){
	  float tempdiffx=state.crioPosition.x-robotAtPathSend.x;
	  float tempdiffy=state.crioPosition.y-robotAtPathSend.y;
	  if(!sentFirst||sqrt(tempdiffx*tempdiffx+tempdiffy*tempdiffy)>0.03){
		  
		CvPoint robotPos;
		robotPos.x=(int)(state.crioPosition.x*(1.0/model->meterToPixel))+translation.x;
		robotPos.y=(int)(state.crioPosition.y*(-1.0/model->meterToPixel))+translation.y;
		
		CvPoint goalpixelpos;
		goalpixelpos.x=(int)(goals[goalIndex ].x*(1.0/model->meterToPixel))+translation.x;
		goalpixelpos.y=(int)(goals[goalIndex ].y*(-1.0/model->meterToPixel))+translation.y;
		CvScalar checkclear;
		CvPoint nextBestGoal;
		bool goal_on_map=false;
		if(goalpixelpos.x>0 && goalpixelpos.y >0&&goalpixelpos.x<SearchImage->width && goalpixelpos.y <SearchImage->height){	
		  checkclear=cvGet2D(SearchImage,goalpixelpos.y,goalpixelpos.x);
		  goal_on_map=true;
		}
		
		bool going_to_actual_point=false;
		if(goal_on_map && checkclear.val[0]<30||checkclear.val[0]>230){
		  cout<<"wee see the goal\n";
		  going_to_actual_point=true;
		  nextBestGoal=goalpixelpos;
		}
		else{
		  nextBestGoal = goalFinder->findBestPoint(frontierCopy,robotPos,state.crioRotRad,goalpixelpos);
		}
	    cout<<"looking for goal "<< goalIndex<<" at "<<goals[goalIndex ].x<<" "<<goals[goalIndex ].y<<" goal pixel "<<goalpixelpos.x<<" "<<goalpixelpos.y<<endl;
		//cvCircle(planImage,nextBestGoal,2,cvScalar(255,255,255),-1,8);
		
		//this will only operate on lidar currently
		float dist_moved=0;
		float max_dist_allowed=1.5;
		path=astar->planImage(SearchImage,robotPos,nextBestGoal,state.crioPosition , model, max_dist_allowed , TimeKeeper::GetTime(),&dist_moved,&killSearch);
		
		cvSetZero(planImage);
		while(!atGoal() && path.empty()&&dist_moved<max_dist_allowed&&!killSearch ){
		  //remove the explored goal from the frontier, if the goal is the actual goal, undiscover it
		  cout<<"impossible goal, need to remove "<<nextBestGoal.x<<" "<<nextBestGoal.y<<endl;
		  if(going_to_actual_point){
			//remove the actual point from explored space
			cvSet2D(SearchImage,nextBestGoal.y,nextBestGoal.x,cvScalar(model->neutral_knowledge,model->neutral_knowledge,model->neutral_knowledge)); 
		  }
		  else{
			// remove the pixel from the frontier map 
			cvSet2D(frontierCopy,nextBestGoal.y,nextBestGoal.x,cvScalar(0,0,0)); 
		  }
			usleep(1000);
		  nextBestGoal = goalFinder->findBestPoint(frontierCopy,robotPos,state.crioRotRad,goalpixelpos);
		  path=astar->planImage(SearchImage,robotPos,nextBestGoal,state.crioPosition , model, max_dist_allowed , TimeKeeper::GetTime(),&dist_moved,&killSearch);

		}
		
		if(!path.empty()){
		  //draw the path and convert it to meters
		  for(int i=0;i<path.size();i++){
			cvCircle(planImage,cvPoint(path[i].x,path[i].y),2,cvScalar(255,255,255),-1,8);
				  cout<<"path "<<path[i].x<<" "<<path[i].y<<endl;
			//convert back to world coodinates
			path[i]=model->pixeltometer(path[i].x,path[i].y);
			cout<<"path "<<path[i].x<<" "<<path[i].y<<endl;
		  }
		  this->trajgen->setPath(path); 
		  robotAtPathSend = model->getstate().crioPosition;
		  sentFirst=true;
		}
	  }
	  
		
	}
	else{
	cout<<"planningover\n"<<endl;

	  pthread_mutex_unlock(& goalMut );
	  return true;
	}
	
	pthread_mutex_unlock(& goalMut );
	usleep(100);
  }
  
  pthread_mutex_unlock(& goalMut );
  return true;
}

//thread function for running the planning thread
void *ThreadGPSPlanner::startThread(void* voidPlanner){
  ((ThreadGPSPlanner*)voidPlanner)->updatePlanner();
  pthread_exit(NULL);
}
