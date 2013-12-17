/*
 * main.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#include <iostream>
#include <cstdlib>
#include <pthread.h>

#include "contact.h"

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
		sw->num_contacts =
				(collide_foot_cylinder_tri(sw->mem, 0.5, 0.3, 0, 0)
						/ TP_CONTACT_CONSTRAINTS * TP_CONTACTS_ON_FOOT);

		*z(tFe(sw->mem, 0)) += -1.0;

//		*x(tFe(sw->mem, 0)) = -10.0;

		step_world(sw->mem, sw->dt, sw->iterations);
	}

	// draw bodies
	for(int i = 0; i < TP_BODIES; ++i) geoms[i]->draw();

	// draw contacts
	draw_contacts(sw);

	// draw bodies' local coord. sys.
	draw_body_refs(sw);
}


int main(int argc, char **argv)
{
	struct mem_t *mem = new mem_t();
	zero_memory(mem);

	BasicWorld w(mem);
	sw = &w;

	create_foot(sw->mem);
	create_foot_geom(sw->mem, geoms);

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
