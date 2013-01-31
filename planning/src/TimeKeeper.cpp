#include "TimeKeeper.h"
#include <stdio.h>

timeval TimeKeeper::start;
timeval TimeKeeper::lastval;
//returns microseconds since we have started

void TimeKeeper::start_time(){
	gettimeofday(&start,0);
	return ;
}

long TimeKeeper::GetTime(){
	gettimeofday(&lastval,0);
	return (long)(lastval.tv_sec*1000000+lastval.tv_usec-start.tv_sec*1000000+start.tv_usec);
}
