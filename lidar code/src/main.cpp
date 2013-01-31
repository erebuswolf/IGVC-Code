#include "Lidar.h"
#include "LidarManager.h"
#include "TimeKeeper.h"
#include <stdio.h>
#include <iostream>
using namespace std;

void wait ( int seconds )
{
  clock_t endwait;
  endwait = clock ()-300000 + seconds * CLOCKS_PER_SEC ;
  while (clock() < endwait) {}
}


int main(int argc, char* argv[])
{
	//Lidar a("lidarconfig.conf");
	//LidarManager b;
	
	printf("wee\n");
	cout<<TimeKeeper::GetTime()<<endl;
	wait(1);
	cout<<TimeKeeper::GetTime()<<endl;
	cout<<TimeKeeper::GetTime()<<endl;
	cout<<TimeKeeper::GetTime()<<endl;
}
