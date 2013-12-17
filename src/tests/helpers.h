/*
 * helpers.h
 *
 *  Created on: Jul 25, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;


inline struct mem_t * stage_memory(bool check_consistency = true)
{
	struct mem_t *m = (struct mem_t *)std::malloc(sizeof(struct mem_t));
	zero_memory(m);

	tp_quatern eq = {1.0, 0.0, 0.0, 0.0};
	tp_mtx33 eR;
	quaternion_to_rot_mtx33(eq, eR);

	for(int b = 0; b < TP_BODIES; ++b)
	{
		set_quatern(eq, quatern(m, b));
		set_mtx33(eR, R(m, b));
	}

	if(check_consistency) assert_consistency(m);

	return m;
}


inline void set_random_J(
		struct mem_t *m,
		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> &rJ)
{
	rJ = Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();

	int s;
	for(s = 0; s < TP_HINGE_MOTOR_CONSTRAINTS; ++s)
	{
		index_t ri = (rand() % TP_BODIES);
		rJ.block(s, ri*6, 1, 6).setRandom();

		*Jm(m, s, 0) = ri;
		*x(tJ(m, s, 0)) = rJ(s, ri*6);
		*y(tJ(m, s, 0)) = rJ(s, ri*6+1);
		*z(tJ(m, s, 0)) = rJ(s, ri*6+2);
		*x(aJ(m, s, 0)) = rJ(s, ri*6+3);
		*y(aJ(m, s, 0)) = rJ(s, ri*6+4);
		*z(aJ(m, s, 0)) = rJ(s, ri*6+5);

		int oldri = ri;
		while(ri == oldri) ri = (rand() % TP_BODIES);
		rJ.block(s, ri*6, 1, 6).setRandom();

		*Jm(m, s, 1) = ri;
		*x(tJ(m, s, 1)) = rJ(s, ri*6);
		*y(tJ(m, s, 1)) = rJ(s, ri*6+1);
		*z(tJ(m, s, 1)) = rJ(s, ri*6+2);
		*x(aJ(m, s, 1)) = rJ(s, ri*6+3);
		*y(aJ(m, s, 1)) = rJ(s, ri*6+4);
		*z(aJ(m, s, 1)) = rJ(s, ri*6+5);
	}

	for(; s < TP_CONSTRAINTS; ++s)
	{
		int ri = (rand() % TP_BODIES);

		rJ.block(s, ri*6, 1, 6).setRandom();

		*Jm(m, s, 0) = -1;
		*Jm(m, s, 1) = ri;
		*x(tJ(m, s, 1)) = rJ(s, ri*6);
		*y(tJ(m, s, 1)) = rJ(s, ri*6+1);
		*z(tJ(m, s, 1)) = rJ(s, ri*6+2);
		*x(aJ(m, s, 1)) = rJ(s, ri*6+3);
		*y(aJ(m, s, 1)) = rJ(s, ri*6+4);
		*z(aJ(m, s, 1)) = rJ(s, ri*6+5);
	}
}


inline void set_random_Mi(struct mem_t *m, Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES> &rMi)
{
	for(int i = 0; i < 6*TP_BODIES; i += 6)
	{
		rMi.block(i, i, 1, 1).setRandom();
		rMi(i+1, i+1) = rMi(i, i);
		rMi(i+2, i+2) = rMi(i, i);
		rMi.block<3, 3>(i+3, i+3).setRandom();

	}
	rMi = rMi.array().abs();

	for(int i = 0, b = 0; i < 6*TP_BODIES; i += 6, ++b)
	{
		*mi(m, b) = rMi(i, i);

		for(int row = 0; row < 3; ++row)
			for(int col = 0; col < 3; ++col)
				*ij(Ibi(m, b), row, col) = rMi(i+3+row, i+3+col);
	}
}


inline void set_random_lambda(struct mem_t *m, Matrix<real_t, TP_CONSTRAINTS, 1> &rlambda)
{
	rlambda = Matrix<real_t, TP_CONSTRAINTS, 1>::Random();

	for(int s = 0; s < TP_CONSTRAINTS; ++s)
		*lambda(m, s) = rlambda(s);
}


inline void set_random_Fe(struct mem_t *m, Matrix<real_t, 6*TP_BODIES, 1> &rFe)
{
	rFe = Matrix<real_t, 6*TP_BODIES, 1>::Random();

	for(int b = 0; b < TP_BODIES; ++b)
	{
		*x(tFe(m, b)) = rFe(b*6 + 0);
		*y(tFe(m, b)) = rFe(b*6 + 1);
		*z(tFe(m, b)) = rFe(b*6 + 2);

		*x(aFe(m, b)) = rFe(b*6 + 3);
		*y(aFe(m, b)) = rFe(b*6 + 4);
		*z(aFe(m, b)) = rFe(b*6 + 5);
	}
}


inline void set_random_v(struct mem_t *m, Matrix<real_t, 6*TP_BODIES, 1> &rv)
{
	rv = Matrix<real_t, 6*TP_BODIES, 1>::Random();

	for(int b = 0; b < TP_BODIES; ++b)
	{
		*x(vel(m, b)) = rv(b*6 + 0);
		*y(vel(m, b)) = rv(b*6 + 1);
		*z(vel(m, b)) = rv(b*6 + 2);

		*x(omega(m, b)) = rv(b*6 + 3);
		*y(omega(m, b)) = rv(b*6 + 4);
		*z(omega(m, b)) = rv(b*6 + 5);
	}
}
