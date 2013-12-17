/*
 * swmem_test.h
 *
 *  Created on: Jul 24, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cxxtest/TestSuite.h>

#define TP_BODIES	3
#define TP_HINGES	1
#define TP_MOTORS	2
#define TP_FEET 	0

#include <tp/tp-core.h>
#include <debugging/debugging.h>
#include <tp/tp.h>

#include "helpers.h"

using namespace Eigen;

class simplemem_test : public CxxTest::TestSuite
{
public:

	/** Tests that it is possible to write and read, all possible locations of
	 * the simulation data, for the memory/simple.h allocation scheme.
	 *
	 * @ingroup tp-tests
	 */
	void fill_world(struct mem_t *m)
	{
		real_t val = 1.0;

		// q
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->q + b*(TP_SIZE_VEC3+TP_SIZE_VEC4);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[TP_SIZE_VEC3] = val++;
			p[TP_SIZE_VEC3+1] = val++;
			p[TP_SIZE_VEC3+2] = val++;
			p[TP_SIZE_VEC3+3] = val++;
		}

		// v
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->v + b*(TP_SIZE_VEC6);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;
		}

		// mi
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->mi + b;

			p[0] = val++;
		}

		// Ibi
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->Ibi + b*(3*TP_SIZE_VEC3);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[TP_SIZE_VEC3+0] = val++;
			p[TP_SIZE_VEC3+1] = val++;
			p[TP_SIZE_VEC3+2] = val++;
			p[2*TP_SIZE_VEC3+0] = val++;
			p[2*TP_SIZE_VEC3+1] = val++;
			p[2*TP_SIZE_VEC3+2] = val++;
		}

		// R
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->R + b*(3*TP_SIZE_VEC3);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[TP_SIZE_VEC3+0] = val++;
			p[TP_SIZE_VEC3+1] = val++;
			p[TP_SIZE_VEC3+2] = val++;
			p[2*TP_SIZE_VEC3+0] = val++;
			p[2*TP_SIZE_VEC3+1] = val++;
			p[2*TP_SIZE_VEC3+2] = val++;
		}

		// Fe
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->Fe + b*(TP_SIZE_VEC6);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;
		}

		// J
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			real_t *p = m->J + c*(2*TP_SIZE_VEC6);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;

			p += TP_SIZE_VEC6;

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;
		}

		// lambda
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			real_t *p = m->lambda + c;

			p[0] = val++;
		}

		// Jm
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			index_t *p = m->Jm + c*2;

			p[0] = val++;
			p[1] = val++;
		}

		// B
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			real_t *p = m->B + c*(2*TP_SIZE_VEC6);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;

			p += TP_SIZE_VEC6;

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;
		}

		// a
		for(int b = 0; b < TP_BODIES; ++b)
		{
			real_t *p = m->a + b*(TP_SIZE_VEC6);

			p[0] = val++;
			p[1] = val++;
			p[2] = val++;
			p[3] = val++;
			p[4] = val++;
			p[5] = val++;
		}

		// d
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			real_t *p = m->d + c;

			p[0] = val++;
		}

		// rhs
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			real_t *p = m->rhs + c;

			p[0] = val++;
		}

		// hanchors
		for(int c = 0; c < TP_HINGES*TP_SIZE_VEC6; ++c)
		{
			real_t *p = m->hanchors + c;

			p[0] = val++;
		}

		// haxes
		for(int c = 0; c < (TP_HINGES)*2*TP_SIZE_VEC6; ++c)
		{
			real_t *p = m->haxes + c;

			p[0] = val++;
		}

		// mm
		for(int c = 0; c < TP_MOTORS; ++c)
		{
			index_t *p = m->mm + c;

			p[0] = val++;
		}

		// mdspeed
		for(int c = 0; c < TP_MOTORS; ++c)
		{
			real_t *p = m->mdspeed + c;

			p[0] = val++;
		}

	}

	/** Tests memory allocation, by visual inspection.
	 *
	 * @ingroup tp-tests
	 */
	void test_print_fill()
	{
		struct mem_t *m = stage_memory(false);
		fill_world(m);
		std::cout << std::endl;

		std::cout << "mdspeed" << std::endl;
		print_vec(m->mdspeed, 1, TP_MOTORS);

		std::cout << "mm" << std::endl;
		print_vec(m->mm, 1, TP_MOTORS);

		std::cout << "haxes" << std::endl;
		print_vec(m->haxes, 2*TP_SIZE_VEC6, TP_HINGES);

		std::cout << "hanchors" << std::endl;
		print_vec(m->hanchors, TP_SIZE_VEC6, TP_HINGES);

		std::cout << "rhs" << std::endl;
		print_vec(m->rhs, 1, TP_CONSTRAINTS);

		std::cout << "d" << std::endl;
		print_vec(m->d, 1, TP_CONSTRAINTS);

		std::cout << "a" << std::endl;
		print_vec(m->a, TP_SIZE_VEC6, TP_BODIES);

		std::cout << "B" << std::endl;
		print_vec(m->B, TP_SIZE_VEC6, 2*TP_CONSTRAINTS);

		std::cout << "Jm" << std::endl;
		print_vec(m->Jm, 2, TP_CONSTRAINTS);

		std::cout << "lambda" << std::endl;
		print_vec(m->lambda, 1, TP_CONSTRAINTS);

		std::cout << "J" << std::endl;
		print_vec(m->J, TP_SIZE_VEC6, 2*TP_CONSTRAINTS);

		std::cout << "Fe" << std::endl;
		print_vec(m->Fe, TP_SIZE_VEC6, TP_BODIES);

		std::cout << "R" << std::endl;
		print_vec(m->R, TP_SIZE_VEC3, 3*TP_BODIES);

		std::cout << "Ibi" << std::endl;
		print_vec(m->Ibi, TP_SIZE_VEC3, 3*TP_BODIES);

		std::cout << "mi" << std::endl;
		print_vec(m->mi, TP_BODIES, 1);

		std::cout << "v" << std::endl;
		print_vec(m->v, TP_SIZE_VEC6, 3);

		std::cout << "q" << std::endl;
		print_vec(m->q, TP_SIZE_VEC3+TP_SIZE_VEC4, 3);


		free(m);
	}

	/** Tests general memory access functions.
	 *
	 * @ingroup tp-tests
	 */
	void test_access_equivalence()
	{
		struct mem_t *m = stage_memory(false);
		fill_world(m);

		TS_ASSERT_DELTA(_x(pos(m, 0)), *x(pos(m, 0)), 1e-7);
		TS_ASSERT_DELTA(_y(pos(m, 0)), *y(pos(m, 0)), 1e-7);
		TS_ASSERT_DELTA(_z(pos(m, 0)), *z(pos(m, 0)), 1e-7);

		TS_ASSERT_DELTA(_q0(quatern(m, 0)), *q0(quatern(m, 0)), 1e-7);
		TS_ASSERT_DELTA(_q1(quatern(m, 0)), *q1(quatern(m, 0)), 1e-7);
		TS_ASSERT_DELTA(_q2(quatern(m, 0)), *q2(quatern(m, 0)), 1e-7);
		TS_ASSERT_DELTA(_q3(quatern(m, 0)), *q3(quatern(m, 0)), 1e-7);

		TS_ASSERT_DELTA(_ij(R(m, 0), 1, 1), *ij(R(m, 0), 1, 1), 1e-7);

		TS_ASSERT_DELTA(_lambda(m, 0), *lambda(m, 0), 1e-7)
		TS_ASSERT_DELTA(_d(m, 0), *d(m, 0), 1e-7)
		TS_ASSERT_DELTA(_rhs(m, 0), *rhs(m, 0), 1e-7)

		TS_ASSERT_DELTA(_Jm(m, 0, 0), *Jm(m, 0, 0), 1e-7)
		TS_ASSERT_DELTA(_Jm(m, 0, 1), *Jm(m, 0, 1), 1e-7)

		free(m);
	}

	/** Tests the memory access implementation unique functions for the memory/simple.h allocation.
	 *
	 * @ingroup tp-tests
	 */
	void test_access()
	{
		struct mem_t *m = stage_memory(false);
		fill_world(m);

		real_t val = 1.0;

		// pos + quatern
		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(pos(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_y(pos(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_z(pos(m, b)), val++, 1e-7)

			TS_ASSERT_DELTA(_q0(quatern(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_q1(quatern(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_q2(quatern(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_q3(quatern(m, b)), val++, 1e-7)
		}

		// vel + omega
		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(vel(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_y(vel(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_z(vel(m, b)), val++, 1e-7)

			TS_ASSERT_DELTA(_x(omega(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_y(omega(m, b)), val++, 1e-7)
			TS_ASSERT_DELTA(_z(omega(m, b)), val++, 1e-7)
		}

		// mi
		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_mi(m, b), val++, 1e-7)
		}

		// Ibi
		for(int b = 0; b < TP_BODIES; ++b)
		{
			for(int k = 0; k < 3; ++k)
			{
				for(int p = 0; p < 3; ++p)
				{
					TS_ASSERT_DELTA(_ij(Ibi(m, b), k, p), val++, 1e-7)
				}
			}
		}

		// R
		for(int b = 0; b < TP_BODIES; ++b)
		{
			for(int k = 0; k < 3; ++k)
			{
				for(int p = 0; p < 3; ++p)
				{
					TS_ASSERT_DELTA(_ij(R(m, b), k, p), val++, 1e-7)
				}
			}
		}

		// Fe
		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(tFe(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(tFe(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(tFe(m, b)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aFe(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aFe(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aFe(m, b)), val++, 1e-7);
		}

		// J
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_x(tJ(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(tJ(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(tJ(m, c, 0)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aJ(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aJ(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aJ(m, c, 0)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(tJ(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(tJ(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(tJ(m, c, 1)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aJ(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aJ(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aJ(m, c, 1)), val++, 1e-7);
		}

		// lambda
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_lambda(m, c), val++, 1e-7);
		}

		// Jm
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_Jm(m, c, 0), val++, 1e-7);
			TS_ASSERT_DELTA(_Jm(m, c, 1), val++, 1e-7);
		}

		// B
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_x(tB(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(tB(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(tB(m, c, 0)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aB(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aB(m, c, 0)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aB(m, c, 0)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(tB(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(tB(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(tB(m, c, 1)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aB(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aB(m, c, 1)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aB(m, c, 1)), val++, 1e-7);
		}

		// a
		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(ta(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(ta(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(ta(m, b)), val++, 1e-7);

			TS_ASSERT_DELTA(_x(aa(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_y(aa(m, b)), val++, 1e-7);
			TS_ASSERT_DELTA(_z(aa(m, b)), val++, 1e-7);
		}

		// d
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_d(m, c), val++, 1e-7);
		}

		// rhs
		for(int c = 0; c < TP_CONSTRAINTS; ++c)
		{
			TS_ASSERT_DELTA(_rhs(m, c), val++, 1e-7);
		}

		free(m);
	}

	/** Tests writing to memory.
	 *
	 * @ingroup tp-tests
	 */
	void test_set_mem()
	{
		struct mem_t *m = stage_memory(false);

		*x(pos(m, 0)) = 0.0;
		*y(pos(m, 0)) = 0.0;
		*z(pos(m, 0)) = 0.0;

		*q0(quatern(m, 0)) = 0.0;
		*q1(quatern(m, 0)) = 0.0;
		*q2(quatern(m, 0)) = 0.0;
		*q3(quatern(m, 0)) = 0.0;

		*ij(Ibi(m, 0), 0, 0) = 0.0;
		*ij(Ibi(m, 0), 0, 1) = 0.0;
		*ij(Ibi(m, 0), 0, 2) = 0.0;

		*ij(Ibi(m, 0), 1, 0) = 0.0;
		*ij(Ibi(m, 0), 1, 1) = 0.0;
		*ij(Ibi(m, 0), 1, 2) = 0.0;

		*ij(Ibi(m, 0), 2, 0) = 0.0;
		*ij(Ibi(m, 0), 2, 1) = 0.0;
		*ij(Ibi(m, 0), 2, 2) = 0.0;

		tp_vec3 vec = {-1.0, -2.0, -3.0};
		tp_quatern qt = {-4.0, -5.0, -6.0, -7.0};

		tp_mtx33 mtx 			= {-8.0,	-9.0,	-10.0};
		mtx[TP_SIZE_VEC3] 		= -11.0;
		mtx[TP_SIZE_VEC3+1] 	= -12.0;
		mtx[TP_SIZE_VEC3+2] 	= -13.0;
		mtx[2*TP_SIZE_VEC3] 	= -14.0;
		mtx[2*TP_SIZE_VEC3+1] 	= -15.0;
		mtx[2*TP_SIZE_VEC3+2] 	= -16.0;

		set_vec3(vec, pos(m, 0));
		set_quatern(qt, quatern(m, 0));
		set_mtx33(mtx, Ibi(m, 0));

		TS_ASSERT_DELTA(_x(pos(m, 0)), vec[0], 1e-7);
		TS_ASSERT_DELTA(_y(pos(m, 0)), vec[1], 1e-7);
		TS_ASSERT_DELTA(_z(pos(m, 0)), vec[2], 1e-7);

		TS_ASSERT_DELTA(_q0(quatern(m, 0)), qt[0], 1e-7);
		TS_ASSERT_DELTA(_q1(quatern(m, 0)), qt[1], 1e-7);
		TS_ASSERT_DELTA(_q2(quatern(m, 0)), qt[2], 1e-7);
		TS_ASSERT_DELTA(_q3(quatern(m, 0)), qt[3], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 0), mtx[0], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 1), mtx[1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 2), mtx[2], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 0), mtx[TP_SIZE_VEC3], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 1), mtx[TP_SIZE_VEC3+1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 2), mtx[TP_SIZE_VEC3+2], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 0), mtx[2*TP_SIZE_VEC3], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 1), mtx[2*TP_SIZE_VEC3+1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 2), mtx[2*TP_SIZE_VEC3+2], 1e-7);

		free(m);
	}

	/** Tests reading from memory.
	 *
	 * @ingroup tp-tests
	 */
	void test_get_mem()
	{
		struct mem_t *m = stage_memory(false);

		*x(pos(m, 0)) = 1.0;
		*y(pos(m, 0)) = 2.0;
		*z(pos(m, 0)) = 3.0;

		*q0(quatern(m, 0)) = 4.0;
		*q1(quatern(m, 0)) = 5.0;
		*q2(quatern(m, 0)) = 6.0;
		*q3(quatern(m, 0)) = 7.0;

		*ij(Ibi(m, 0), 0, 0) = 8.0;
		*ij(Ibi(m, 0), 0, 1) = 9.0;
		*ij(Ibi(m, 0), 0, 2) = 10.0;

		*ij(Ibi(m, 0), 1, 0) = 11.0;
		*ij(Ibi(m, 0), 1, 1) = 12.0;
		*ij(Ibi(m, 0), 1, 2) = 13.0;

		*ij(Ibi(m, 0), 2, 0) = 14.0;
		*ij(Ibi(m, 0), 2, 1) = 15.0;
		*ij(Ibi(m, 0), 2, 2) = 16.0;

		tp_vec3 vec;
		tp_quatern qt;

		tp_mtx33 mtx;

		get_vec3(pos(m, 0), vec);
		get_quatern(quatern(m, 0), qt);
		get_mtx33(Ibi(m, 0), mtx);

		TS_ASSERT_DELTA(_x(pos(m, 0)), vec[0], 1e-7);
		TS_ASSERT_DELTA(_y(pos(m, 0)), vec[1], 1e-7);
		TS_ASSERT_DELTA(_z(pos(m, 0)), vec[2], 1e-7);

		TS_ASSERT_DELTA(_q0(quatern(m, 0)), qt[0], 1e-7);
		TS_ASSERT_DELTA(_q1(quatern(m, 0)), qt[1], 1e-7);
		TS_ASSERT_DELTA(_q2(quatern(m, 0)), qt[2], 1e-7);
		TS_ASSERT_DELTA(_q3(quatern(m, 0)), qt[3], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 0), mtx[0], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 1), mtx[1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 0, 2), mtx[2], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 0), mtx[TP_SIZE_VEC3], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 1), mtx[TP_SIZE_VEC3+1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 1, 2), mtx[TP_SIZE_VEC3+2], 1e-7);

		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 0), mtx[2*TP_SIZE_VEC3], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 1), mtx[2*TP_SIZE_VEC3+1], 1e-7);
		TS_ASSERT_DELTA(_ij(Ibi(m, 0), 2, 2), mtx[2*TP_SIZE_VEC3+2], 1e-7);

		free(m);
	}
};

