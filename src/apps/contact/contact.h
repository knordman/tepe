/*
 * contact.h
 *
 *  Created on: Aug 1, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#define TP_DEBUG

#define TP_BODIES		1
#define TP_HINGES		0
#define TP_MOTORS		0
#define TP_FEET			1

#include <tp/tp-core.h>
#include <tp/debugging.h>
#include <tp/tp.h>

#include <apps/common/util/Geom.h>

inline void create_foot_geom(struct mem_t *m, Geom *geoms[])
{
	real_t h = 0.3, r = 0.5;

	Cylinder *c = new Cylinder(r, h, pos(m, 0), R(m, 0));
	geoms[0] = c;
}

inline void create_foot(struct mem_t *m)
{
	tp_quatern eq = {1.0, 0.0, 0.0, 0.0};

	tp_mtx33 Re;
	quaternion_to_rot_mtx33(eq, Re);
	set_mtx33(Re, R(m, 0));
	set_quatern(eq, quatern(m, 0));

	// CYLINDER -----------------------------------------------------

	*x(pos(m, 0)) = -1.0;
	*y(pos(m, 0)) = -1.0;
	*z(pos(m, 0)) = 0.5;

	real_t h = 0.3, r = 0.5;

	set_cylinder_inertia(1.0, mi(m, 0), r, h, Ibi(m, 0));
}
