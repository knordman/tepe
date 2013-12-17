/*
 * timit.h
 *
 *  Created on: Jul 6, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#pragma once

#ifdef __APPLE__

	#include <stdint.h>
	#include <mach/mach_time.h>

	typedef uint64_t time_stamp_t;

#elif defined WIN64 || defined WIN32 || defined __CYGWIN__

	#include <Windows.h>

	typedef LARGE_INTEGER time_stamp_t;

#elif defined __unix__

	#include <sys/time.h>

	typedef struct timeval time_stamp_t;

#else
	typedef int time_stamp_t;
#endif

time_stamp_t get_time_stamp();
double time_since_ms(time_stamp_t time_stamp);

time_stamp_t get_time_stamp();
double time_since_ms(time_stamp_t time_stamp);
