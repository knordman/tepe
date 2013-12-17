/*
 * main.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#include <iostream>
#include <cstdlib>
#include <pthread.h>

#include "hinge.h"

#include <apps/common/util/world/BasicWorld.h>
#include <apps/common/util/drawing.h>
#include <apps/common/util/parser/parser_def.h>

static BasicWorld *sw;
static Geom *geoms[TP_BODIES];
static dsFunctions fn;


void * interactive_console(void *data)
{
	World *w = (World *)data;

	while(true) yyparse(w);

	return NULL;
}


void sim_loop(int pause)
{
	if(!pause)
	{
		step_world(sw->mem, sw->dt, sw->iterations);

//		real_t rate = hinge_angle_rate(sw->mem, 0);
//		real_t angle = hinge_angle(sw->mem, 0);
//		real_t torque = motor_torque(sw->mem, 0);
//
//		std::cout << std::setprecision(4);
//
//		std::cout << "rate: " << std::setw(10) << rate;
//		std::cout << "   torque: " << std::setw(10) << torque;
//		std::cout << "   angle: " << std::setw(10) << angle * 57.2957795;
//		std::cout << std::endl;
	}

	// draw bodies
	for(int i = 0; i < TP_BODIES; ++i) geoms[i]->draw();

	// draw hinges
	draw_hinges(sw);

	// draw bodies' local coord. sys.
	draw_body_refs(sw);
}


int main(int argc, char **argv)
{
	struct mem_t *mem = new mem_t();
	zero_memory(mem);

	BasicWorld w(mem);
	sw = &w;

	create_hinge_bodies(sw->mem);
	create_hinge_bodies_geom(sw->mem, geoms);

	assert_consistency(sw->mem);

	pthread_t console_thread;
	pthread_create(&console_thread, NULL, interactive_console, (void *)&w);

	fn.version = DS_VERSION;
	fn.start = NULL;
	fn.command = NULL;
	fn.step = sim_loop;
	fn.path_to_textures = "./src/apps/common/util/drawstuff/textures";
	dsSimulationLoop(argc, argv, 800, 600, &fn);

	delete mem;

	for(int g = 0; g < TP_BODIES; ++g) delete geoms[g];

	return EXIT_SUCCESS;
}
