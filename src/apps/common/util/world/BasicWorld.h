/*
 * ContactWorld.h
 *
 *  Created on: Aug 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include "World.h"

class BasicWorld : public World
{
public:
	BasicWorld(struct mem_t *mem)
	: draw_contacts(true),
	  mem(mem)
	{
		for(int h = 0; h < (TP_HINGES); ++h)
			draw_hinges[h] = false;

		for(int b = 0; b < (TP_BODIES); ++b)
		{
			draw_body_reframes[b] = false;

			std::stringstream label;
			label << "fe body " << b << " x";
			add_interactive_property(label.str(), mem->Fe[b*TP_SIZE_VEC6]);

			label.str(""); label << "fe body " << b << " y";
			add_interactive_property(label.str(), mem->Fe[b*TP_SIZE_VEC6+1]);

			label.str(""); label << "fe body " << b << " z";
			add_interactive_property(label.str(), mem->Fe[b*TP_SIZE_VEC6+2]);
		}

		add_interactive_readonly_array_property("q", mem->q, TP_SIZE_VEC6, (TP_BODIES)*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("v", mem->v, TP_SIZE_VEC6, (TP_BODIES)*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("mi", mem->mi, 1, (TP_BODIES));
		add_interactive_readonly_array_property("Ibi", mem->Ibi, TP_SIZE_VEC3, (TP_BODIES)*3*TP_SIZE_VEC3);
		add_interactive_readonly_array_property("R", mem->R, TP_SIZE_VEC3, (TP_BODIES)*3*TP_SIZE_VEC3);

		add_interactive_readonly_array_property("Fe", mem->Fe, TP_SIZE_VEC6, (TP_BODIES)*TP_SIZE_VEC6);

		add_interactive_readonly_array_property("J", mem->J, 2*TP_SIZE_VEC6, TP_CONSTRAINTS*2*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("Jm", mem->Jm, 2, 2*TP_CONSTRAINTS);

		add_interactive_readonly_array_property("lambda", mem->lambda, 1, TP_CONSTRAINTS);
		add_interactive_readonly_array_property("lambda min", mem->lambda_min, 1, TP_CONSTRAINTS);
		add_interactive_readonly_array_property("lambda max", mem->lambda_max, 1, TP_CONSTRAINTS);

		add_interactive_readonly_array_property("motor map", mem->mm, 1, (TP_MOTORS));
		add_interactive_readonly_array_property("motor desired speed", mem->mdspeed, 1, (TP_MOTORS));

		add_interactive_readonly_array_property("quatern initial rot", mem->iniq, TP_SIZE_VEC4, (TP_HINGES)*TP_SIZE_VEC4);

		add_interactive_readonly_array_property("B", mem->B, 2*TP_SIZE_VEC6, TP_CONSTRAINTS*2*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("a", mem->a, TP_SIZE_VEC6, (TP_BODIES)*TP_SIZE_VEC6);

		add_interactive_readonly_array_property("d", mem->d, 1, TP_CONSTRAINTS);
		add_interactive_readonly_array_property("rhs", mem->rhs, 1, TP_CONSTRAINTS);

		add_interactive_readonly_array_property("haxes", mem->haxes, 2*TP_SIZE_VEC6, (TP_HINGES)*2*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("hanchors", mem->hanchors, TP_SIZE_VEC6, (TP_HINGES)*TP_SIZE_VEC6);

#ifdef TP_DEBUG
		add_interactive_readonly_array_property("Fc", mem->Fc, TP_SIZE_VEC6, (TP_BODIES)*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("cinfo", mem->cinfo, TP_SIZE_VEC6, (TP_FEET)*3*TP_SIZE_VEC6);
		add_interactive_readonly_array_property("cplane", mem->cplane, TP_SIZE_VEC3, (TP_FEET)*TP_SIZE_VEC3);
#endif

		for(int h = 0; h < (TP_HINGES); ++h)
		{
			std::stringstream label;
			label << "draw hinge " << h;
			add_interactive_property(label.str(), draw_hinges[h]);
		}

		for(int b = 0; b < (TP_BODIES); ++b)
		{
			std::stringstream label;
			label << "draw local frame " << b;
			add_interactive_property(label.str(), draw_body_reframes[b]);
		}

		add_interactive_property("draw contacts", draw_contacts);
		add_interactive_readonly_array_property("draw hinges", draw_hinges, 1, (TP_HINGES));
		add_interactive_readonly_array_property("draw body frames", draw_body_reframes, 1, (TP_BODIES));
	}

	bool draw_hinges[(TP_HINGES)];
	bool draw_body_reframes[(TP_BODIES)];
	bool draw_contacts;

	struct mem_t *mem;
};
