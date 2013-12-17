/*
 * TitanWorld.h
 *
 *  Created on: Aug 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include "BasicWorld.h"


class TitanVIIIWorld : public BasicWorld
{
public:
	TitanVIIIWorld(
			struct mem_t *mem,
			real_t period,
			real_t stride)
	: BasicWorld(mem),
	  gait_period(period),
	  gait_stride(stride),
	  gait_start_time(1.0)
	{
		for(int j = 0; j < 16; ++j)
		{
			desired_joint_angles[j] = 0.0;
			std::stringstream label;
			label << "ref j " << j;
			add_interactive_property(label.str(), desired_joint_angles[j]);
		}

		add_interactive_property("gait start time", gait_start_time);
		add_interactive_property("gait period", gait_period);
		add_interactive_property("gait stride", gait_stride);
	}

	real_t gait_period;
	real_t gait_stride;
	real_t gait_start_time;
	real_t desired_joint_angles[16];
	time_stamp_t time_stamp;
};


