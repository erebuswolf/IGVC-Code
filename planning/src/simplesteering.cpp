
#include "simplesteering.h"
#include <math.h>
#include <stdio.h>
/*
diff x 0.901652, diff y -1.717779, angle2.626226
sending vel command vel 0.582011, omega 0.835978
psoition data -0.901819 1.7183 2.56934
sending goal point
diff x 0.901819, diff y -1.718301, angle2.626177
sending vel command vel 0.582173, omega 0.835963
capture thread killd
*/
AngularRateHolder simplesteering(float x, float y, PositionState state,float vel_slope, float vel_sat,float omega_slope, float omega_sat,float theta_vel_thresh){
	AngularRateHolder value;
	float diffx=x-state.crioPosition.x;
	float diffy=y-state.crioPosition.y;
	float distance_to_goal=sqrt(diffx*diffx+diffy*diffy);
	
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
	//trying new values
	value.omega=sign_of_omega*min(abs(1.2*angle_to_goal),.9);
	
	//try this and see how it goes
	//convert rad to deg to inhibit by angle more
	value.vel= min( (0.3+(distance_to_goal*0.05))*0.1/abs(angle_to_goal),.9);
		
	
	/*
	if(abs(angle_to_goal)<theta_vel_thresh){
//	vel_sat=vel_sat - vel_sat*(abs(angle_to_goal*omega_vel_influence)/(pi));
		value.vel=min(distance_to_goal*vel_slope,vel_sat);
	}
	else{
		value.vel=.4/(abs(angle_to_goal)/pi);
	}
	
	value.omega=sign_of_omega*min(abs(omega_slope*angle_to_goal),omega_sat);
	printf("diff x %f, diff y %f, angle%f\n",diffx,diffy,angle_to_goal);
	*/
	return value;
}
