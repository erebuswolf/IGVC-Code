
#include "TrajectoryGenerator.h"
#include "simplesteering.h"
#include <math.h>

TrajectoryGenerator::TrajectoryGenerator(float success_dist,bool post_process){
  this->post_process=post_process;
  this->success_dist= success_dist;
  pthread_mutex_init(&(this->pathMut),NULL);
  newpath=false;
}
TrajectoryGenerator::~TrajectoryGenerator(){
  
}

void TrajectoryGenerator::setPath(vector <CvPoint2D32f> path){
  pthread_mutex_lock(&pathMut);
  this->path=path;
  pathIndex=0;
  newpath=true;
  pthread_mutex_unlock(&pathMut);
}
bool TrajectoryGenerator::gotNewPath(){
  bool val;
//  changinggoal=true
  pthread_mutex_lock(&pathMut);
  val=newpath;
  newpath=false;
  pthread_mutex_unlock(&pathMut);
  return val;
}

bool TrajectoryGenerator::updateRobot(WorldModel * model, int steps_ahead){
if(model->inDanger){
//  model->network->sendWarning();
  //bail on plan
  pathIndex=path.size();  
  return false;
}else{
  pthread_mutex_lock(&pathMut);
  if(path.size()>0){
	float diffx=0;
	float diffy=0;
	
	int start=pathIndex;
	for(int i=pathIndex;i<path.size() && pathIndex<path.size(); i++){
	  diffx=model->state.crioPosition.x-path[i].x;
	  diffy=model->state.crioPosition.y-path[i].y;
	  cout<<"checking path "<<path[i].x<<" "<<path[i].y<<endl;
	  if(sqrt(diffx*diffx+diffy*diffy) <success_dist){
		pathIndex++;
	  }
	  else if(i-start>steps_ahead){
		//if we have tried 4 steps and not found anything break
		//if we found something but then lost it also break
		break;
	  }
	}
	cout<<"commanging to  "<<pathIndex<<" "<<path[pathIndex].x<<" "<<path[pathIndex].y<<endl;
		 
	if(!post_process){
	  if(model->network!=NULL){
		if(pathIndex<path.size()){
		
		  AngularRateHolder temp =trajSteering(model->state);
		  
		  cout<<pathIndex<<" "<<path.size()<<endl;
		  cout<<"command "<<temp.vel<<" "<<temp.omega<<endl;
		  
		  model->network->angularRateSpeedCommand( temp.omega,temp.vel);
		}
		else{
//		  model->network->sendWarning();
		  //pathIndex--;
		  //AngularRateHolder temp =trajSteering(model->state);
		//  cout<<"at goal, so sending strong turn "<<temp.omega/abs(temp.omega)*max(abs(temp.omega),.6f)<<endl;
		 // model->network->angularRateSpeedCommand(temp.omega/abs(temp.omega)*max(abs(temp.omega),1.0f) ,0.3);

		  //pathIndex++;
		  pthread_mutex_unlock(&pathMut);
		  return false;
		}
	  }
	  else{
		cout<<"error network is null\n";
	  }
	}
	else{
	}
  }
  pthread_mutex_unlock(&pathMut);
  }
  return true;
}

AngularRateHolder TrajectoryGenerator::trajSteering(PositionState state ){
  int dist_to_end=path.size()-pathIndex;

  AngularRateHolder value;
  float diffx=path[ pathIndex ].x-state.crioPosition.x;
  float diffy=path[ pathIndex ].y-state.crioPosition.y;
  float distance_to_goal=sqrt(diffx*diffx+diffy*diffy);
  int pathenddist=path.size()-pathIndex;
  float pi =3.1415;

  //minus pi/2 because our coordinate frame has 0 at the y axis
  float angle_of_goal=atan2(diffy,diffx)-(pi/2);

  while(angle_of_goal<0){
  angle_of_goal+=2*pi;
  }

  float angle_to_goal=angle_of_goal-state.crioRotRad;

  if(angle_to_goal<-pi){
	angle_to_goal+=2*pi;
  }
  else if(angle_to_goal>pi){
	angle_to_goal-=2*pi;
  }
  
  int sign_of_omega=round(angle_to_goal/abs(angle_to_goal));
  
   value.omega=sign_of_omega*min(abs(1.2*angle_to_goal),.7);
	
	//try this and see how it goes
	//convert rad to deg to inhibit by angle more
   value.vel= min( (0.1+(pathenddist*0.01))*4/abs(angle_to_goal),.9);
  return value;
  
}
