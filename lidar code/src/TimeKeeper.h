#ifndef _Time_Keeper_h
#define _Time_Keeper_h

#include <time.h>

/// Simple class to assure all time stamps are in the same reference frame
class TimeKeeper{
	private:
		static clock_t start;
	public:
		static double GetTime();
};

#endif
