/**
*	main.cpp
*	Author: Jesse Fish
* 
* Sandbox for testing mostly
*
*/

#include <stdio.h>
#include <iostream>
#include "observermode.h"
#include "pointandclick.h"
#include "recorder.h"
#include "lidarlocalizorapp.h"
#include "wallcrawling.h"
#include "bugging.h"
#include "birdseyecalibrator.h"
#include "intrinsiccalibrator.h"
#include "PostProcess.h"
#include "GoalFinder.h"
#include "doGPS.h"
#include "AStar.h"
#include "qualifier.h"
#include "NetworkStack.h"

using namespace std;

int prompt();
void go(){
	Network net;
	net.angularRateSpeedCommand(0.0,2.0);
}
	

int main(int argc, char* argv[])
{
	int b=0;
	while(1){
		switch(prompt()){
			case 1:
				printf("Observer Mode\n");
				observermode();
				return 0;
			case 2:
				printf("Recording mode\n");
				recorder();
				return 0;
			case 3:
				printf("Point and Click Steering\n");
				pointandclick();
				return 0;
			case 4:
				printf("wall crawling\n");
				wallcrawling();
				return 0;
			case 5:
				printf("Post Process Log Files\n");
				postprocess();
				return 0;
			case 6:
				printf("Camera Calibration\n");
				printf("1. calibrate intrinsics\n2. calibrate warp\n");
				cin>>b;
				switch(b){
				  case 1:
					intrinsiccalibrator(8, 6, 20,2.85);
					return 0;
				  case 2:
				  //7feet =213.36 , 8feet= 243.84 6' 183.04
					birdseyecalibrator(7, 6, "vision/calibration/Intrinsic.xml","vision/calibration/Distortion.xml", 11.1, 5, 183.04,600, 600);
					return 0;
				  default:
					printf("invalid selection\n");
					break;
				}
				return 0;
			case 7:
				printf("IGVC Path Following\n");
				printf("Not Yet Implimented\n");
				return 0;
			case 8:
			//	printf("IGVC Waypoint\n");
				printf("Bugging\n");
				bugging();
				return 0;
			case 9:
				printf("Starting indoor lidar localization\n");
				lidarlocalizorapp();
			case 10:
				printf("testing goal finding\n");
				doGPS();
				return 0;
				//testGoalFinding();
			case 11:
				printf("qualling\n");
				
				qualifier();
				return 0;
			case 66:
				//go();
				return 0;
			case 0:
				printf("Quitting\n");
				return 0;
			default:
				printf("invalid selection\n");
				break;
		}
	}
	
}

int prompt(){
	int a=-1;
	printf(\
	"What mode would you like to run in?\n1. Observer mode\n2. Recording mode\n3. Point and Click Steering\n4. Wall Crawling\n5. Post Process Log Files\n6. Camera Calibration\n7. IGVC Path Following\n8. Bugging (igvc goal find)\n9. indoor lidar\n10. test goal finding\n11. qual\n0. Quit\n\n");
	cin>>a;
	return a;
}
