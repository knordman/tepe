/*
 * geom.h
 *
 *  Created on: Jul 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#pragma once

// Since pos and R are passed straight off, this assumes sw_mem allocation
// TODO: change so that it instantenously uses get_vec3() etc...

#include <apps/common/util/drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox dsDrawBoxD
#endif

class Geom
{
public:
	virtual void draw() = 0;
	virtual ~Geom() {};
};

class Cylinder : public Geom
{
private:
	float radius;
	float height;
	real_t *pos;
	real_t *R;
public:
	Cylinder(float radius, float height, real_t *pos, real_t *R) :
		radius(radius), height(height),
		pos(pos), R(R) {};
	void draw()
	{
		dsDrawCylinder(pos, R, height, radius);
	}
};

class Box : public Geom
{
private:
	real_t sides[3];
	real_t *pos;
	real_t *R;
public:
	Box(const tp_vec3 sides, real_t *pos, real_t *R) :
		pos(pos), R(R)
		{
			this->sides[0] = sides[0];
			this->sides[1] = sides[1];
			this->sides[2] = sides[2];
		};
	Box(real_t xlen, real_t ylen, real_t zlen, real_t *pos, real_t *R) :
		pos(pos), R(R)
		{
			sides[0] = xlen;
			sides[1] = ylen;
			sides[2] = zlen;
		};
	void draw()
	{
		dsDrawBox(pos, R, sides);
	}
};
