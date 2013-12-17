/*
 * timit.cpp
 *
 *  Created on: Jul 6, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#include "timeit.h"


#ifdef __APPLE__

time_stamp_t get_time_stamp()
{
	return mach_absolute_time();
}

double time_since_ms(time_stamp_t time_stamp)
{
	time_stamp_t now = mach_absolute_time();

	mach_timebase_info_data_t info;
	mach_timebase_info(&info);

	time_stamp_t elapsed = now - time_stamp;

	return ((double)elapsed * (double)info.numer / (double)info.denom)/1e6;
}

#elif defined WIN64 || defined WIN32 || defined __CYGWIN__

time_stamp_t get_time_stamp()
{
	LARGE_INTEGER stamp;
	QueryPerformanceCounter(&stamp);
	return stamp;
}

double time_since_ms(time_stamp_t time_stamp)
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	LONGLONG time_diff = now.QuadPart - time_stamp.QuadPart;

	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);

	return (double)time_diff * 1e3 / (double)frequency.QuadPart;
}

#elif defined __unix__

time_stamp_t get_time_stamp()
{
	struct timeval stamp;
	gettimeofday(&stamp, NULL);
	return stamp;
}

double time_since_ms(time_stamp_t time_stamp)
{
	struct timeval now, diff;
	gettimeofday(&now, NULL);

	/* From:
	 * http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	 */
    if (now.tv_usec < time_stamp.tv_usec) {
        int nsec = (time_stamp.tv_usec - now.tv_usec) / 1000000 + 1;
        time_stamp.tv_usec -= 1000000 * nsec;
        time_stamp.tv_sec += nsec;
    }
    if (now.tv_usec - time_stamp.tv_usec > 1000000) {
        int nsec = (now.tv_usec - time_stamp.tv_usec) / 1000000;
        time_stamp.tv_usec += 1000000 * nsec;
        time_stamp.tv_sec -= nsec;
    }
     
    /* Compute the time remaining to wait.
        tv_usec is certainly positive. */
    diff.tv_sec = now.tv_sec - time_stamp.tv_sec;
    diff.tv_usec = now.tv_usec - time_stamp.tv_usec;

	return (double)(diff.tv_sec*1000) + (double)diff.tv_usec / 1000.0;
}


#else

time_stamp_t get_time_stamp()
{
	return 0;
}

double time_since_ms(time_stamp_t time_stamp)
{
	return 0.0;
}

#endif

