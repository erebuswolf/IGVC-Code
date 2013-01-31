#include "TimeKeeper.h"
#include <stdio.h>
clock_t TimeKeeper::start=clock();

double TimeKeeper::GetTime(){
	return (double(clock()-start))/CLOCKS_PER_SEC;
}
