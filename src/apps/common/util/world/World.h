/*
 * World.h
 *
 *  Created on: Aug 21, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#pragma once

#include <tp/types/default.h>
#include <apps/common/util/configurable/Configurable.h>

struct mem_t;

class World : public Configurable
{
public:
	World()
	: Configurable("simulation"),
	  iterations(200),
	  dt(0.005),
	  sim_time(0.0)
	{
		add_interactive_property("dt", dt);
		add_interactive_property("iterations", iterations);
		add_interactive_property("sim time", sim_time);
	}
	virtual ~World(){};

	int iterations;
	real_t dt;
	real_t sim_time;

	int num_contacts;
};

