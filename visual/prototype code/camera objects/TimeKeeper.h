#ifndef _Time_Keeper_h
#define _Time_Keeper_h

#include <sys/time.h>

/// Simple class to assure all time stamps are in the same reference frame
class TimeKeeper{
	private:
		static timeval start;
		static timeval lastval;
	public:
		static void start_time();
		static long GetTime();
};

#endif
