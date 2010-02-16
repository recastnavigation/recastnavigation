#include "RecastTimer.h"

#if defined(WIN32)

// Win32
#include <windows.h>

rcTimeVal rcGetPerformanceTimer()
{
	__int64 count;
	QueryPerformanceCounter((LARGE_INTEGER*)&count);
	return count;
}

int rcGetDeltaTimeUsec(rcTimeVal start, rcTimeVal end)
{
	static __int64 freq = 0;
	if (freq == 0)
		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
	__int64 elapsed = end - start;
	return (int)(elapsed*1000000 / freq);
}

#else

// Linux, BSD, OSX

#include <sys/time.h>

rcTimeVal rcGetPerformanceTimer()
{
	timeval now;
	gettimeofday(&now, 0);
	return (rcTimeVal)now.tv_sec*1000000L + (rcTimeVal)now.tv_usec;
}

int rcGetDeltaTimeUsec(rcTimeVal start, rcTimeVal end)
{
	return (int)(end - start);
}

#endif
