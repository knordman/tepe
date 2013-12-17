/*
 * contact.h
 *
 *  Created on: Aug 1, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#define TP_DEBUG

#define TP_BODIES		2
#define TP_HINGES		1
#define TP_MOTORS		1
#define TP_FEET			0

#define TP_DEBUG

#include <tp/tp-core.h>
#include <tp/debugging.h>
#include <tp/tp.h>

#include <apps/common/util/Geom.h>

inline void create_hinge_bodies_geom(struct mem_t *m, Geom *geoms[])
{
	real_t xlen = 0.5, ylen = 0.5, zlen = 1.5;
	set_box_inertia(5.0, mi(m, 0), xlen, ylen, zlen, Ibi(m, 0));

	Box *b0 = new Box(xlen, ylen, zlen, pos(m, 0), R(m, 0));
	geoms[0] = b0;

	xlen = 0.5; ylen = 0.5; zlen = 0.5;
	set_box_inertia(1.0, mi(m, 1), xlen, ylen, zlen, Ibi(m, 1));

	Box *b1 = new Box(xlen, ylen, zlen, pos(m, 1), R(m, 1));

	geoms[1] = b1;
}


inline void create_hinge_bodies(struct mem_t *m)
{
	tp_mtx33 R0, R1;

	tp_quatern eq = {1.0, 0.0, 0.0, 0.0};
	tp_quatern eq1 = {0.707107, 0, -0.707107, 0};
//	tp_quatern eq1 = {0.965926, 0.0, 0.0, -0.258819};
//	tp_quatern eq1 = {0.923879, -0.382684, 0.0, 0.0};
//	tp_quatern eq1 = {0.923879, 0.0, -0.382684, 0.0};
//	tp_quatern eq1 = {0.923879, 0.0, 0.0, -0.382684};
//	tp_quatern eq1 = {1.0, 0.0, 0.0, 0.0};

	quaternion_to_rot_mtx33(eq, R0);
	set_mtx33(R0, R(m, 0));
	set_quatern(eq, quatern(m, 0));

	quaternion_to_rot_mtx33(eq1, R1);
	set_mtx33(R1, R(m, 1));
	set_quatern(eq1, quatern(m, 1));

	// BOX 1 --------------------------------------------------------
	*x(pos(m, 0)) = 0.0;
	*y(pos(m, 0)) = -0.5;
	*z(pos(m, 0)) = 1.0;

	real_t xlen = 0.5, ylen = 0.5, zlen = 1.5;
	set_box_inertia(15.0, mi(m, 0), xlen, ylen, zlen, Ibi(m, 0));
	// END OF BOX 1, BOX2 ---------------------------------------------
	*x(pos(m, 1)) = 0.0;
	*y(pos(m, 1)) = 0.5;
	*z(pos(m, 1)) = 1.0;

	xlen = 0.5; ylen = 0.5; zlen = 0.5;
	set_box_inertia(1.0, mi(m, 1), xlen, ylen, zlen, Ibi(m, 1));
	// END OF BOX 2 ------------------------------------------------

	tp_vec3 anw = {0.5, 0.0, 1.0};
	tp_vec3 axw = {1.0, 0.0, 0.0};

	create_hinge(m, 0, 0, 1, anw, axw);

	add_motor(m, 0, 0, 0.2);

	*mds(m, 0) = 1.0;
}
