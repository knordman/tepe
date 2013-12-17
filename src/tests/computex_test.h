/*
 * solver_test.h
 *
 *  Created on: Jul 24, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cxxtest/TestSuite.h>

#define TP_DEBUG

// Tests below relies on these values
#define TP_BODIES	3
#define TP_HINGES	1
#define TP_MOTORS	1
#define TP_FEET 	1
#define TP_ERP 		0.0

#include <tp/tp-core.h>
#include <debugging/debugging.h>
#include <tp/tp.h>

#include "helpers.h"


class computex_test : public CxxTest::TestSuite
{
public:

	/** Tests computing constraint forces, see \ref tp-dynamics.
	 *
	 * @ingroup tp-tests
	 */
	void test_compute_Fc()
	{
		struct mem_t *m = stage_memory();

		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> rJ = Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();
		set_random_J(m, rJ);

		Matrix<real_t, TP_CONSTRAINTS, 1> rlambda;
		set_random_lambda(m, rlambda);

		Matrix<real_t, 6*TP_BODIES, 1> Fc = rJ.transpose() * rlambda;
		compute_Fc(m);

//		std::cout << Fc << std::endl;
//
//		print_vec(m->Fc, 6, TP_BODIES);

		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(tFc(m, b)), Fc(b*6), 1e-7);
			TS_ASSERT_DELTA(_y(tFc(m, b)), Fc(b*6+1), 1e-7);
			TS_ASSERT_DELTA(_z(tFc(m, b)), Fc(b*6+2), 1e-7);

			TS_ASSERT_DELTA(_x(aFc(m, b)), Fc(b*6+3), 1e-7);
			TS_ASSERT_DELTA(_y(aFc(m, b)), Fc(b*6+4), 1e-7);
			TS_ASSERT_DELTA(_z(aFc(m, b)), Fc(b*6+5), 1e-7);
		}

		free(m);
	}

	/** Tests computing \f$B\f$ and \f$a\f$ vectors, see \ref tp-dynamics.
	 *
	 * @ingroup tp-tests
	 */
	void test_compute_B_and_a()
	{
		struct mem_t *m = stage_memory();

		Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES> rMi	= Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES>::Zero();
		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> rJ	= Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();

		set_random_J(m, rJ);
		set_random_Mi(m, rMi);

		Matrix<real_t, 6*TP_BODIES, TP_CONSTRAINTS> rB = rMi * (rJ.transpose());
		compute_B(m);

//		std::cout << std::endl;
//		std::cout << "B" << std::endl;
//		print_vec(w->mem->B, 2*TP_SIZE_VEC6, TP_CONSTRAINTS);
//
//		std::cout << "Jm" << std::endl;
//		print_vec(w->mem->Jm, 2, TP_CONSTRAINTS);
//
//		std::cout << rB << std::endl;

		Matrix<real_t, TP_CONSTRAINTS, 1> rlambda;
		set_random_lambda(m, rlambda);

		Matrix<real_t, 6*TP_BODIES, 1> ra = rB * rlambda;
		compute_a(m);

		for(int b = 0; b < TP_BODIES; ++b)
		{
			TS_ASSERT_DELTA(_x(ta(m, b)), ra(b*6), 1e-7);
			TS_ASSERT_DELTA(_y(ta(m, b)), ra(b*6+1), 1e-7);
			TS_ASSERT_DELTA(_z(ta(m, b)), ra(b*6+2), 1e-7);

			TS_ASSERT_DELTA(_x(aa(m, b)), ra(b*6+3), 1e-7);
			TS_ASSERT_DELTA(_y(aa(m, b)), ra(b*6+4), 1e-7);
			TS_ASSERT_DELTA(_z(aa(m, b)), ra(b*6+5), 1e-7);
		}

		free(m);
	}

	/** Tests computing the diagonal \f$d\f$ of \f$JB\f$, see \ref tp-dynamics.
	 *
	 * @ingroup tp-tests
	 */
	void test_compute_d()
	{
		struct mem_t *m = stage_memory();

		Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES> rMi 	= Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES>::Zero();
		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> rJ 	= Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();

		set_random_J(m, rJ);
		set_random_Mi(m, rMi);

		Matrix<real_t, 6*TP_BODIES, TP_CONSTRAINTS> rB = rMi * (rJ.transpose());
		compute_B(m);

		Matrix<real_t, TP_CONSTRAINTS, TP_CONSTRAINTS> rJB = rJ * rB;
		compute_d(m);

		Matrix<real_t, TP_CONSTRAINTS, 1> rd = rJB.diagonal();

//		std::cout << std::endl;
//		std::cout << rd << std::endl;
//		print_vec(m->d, 1, TP_CONSTRAINTS);

		for(int s = 0; s < TP_CONSTRAINTS; ++s)
			TS_ASSERT_DELTA(rd(s), _d(m, s), 1e-7);

		free(m);
	}

	/** Tests computing \f$rhs\f$, see \ref tp-dynamics.
	 *
	 * @ingroup tp-tests
	 */
	void test_compute_rhs()
	{
		struct mem_t *m = stage_memory();

		/* Eigen strange: if matrices not initiated, other functions will affect
		 * the local variables? No. Same memory is allocated, but this time it is
		 * not zero, so erroneous values are introduced in the matrices
		 */

		Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES> rMi 	= Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES>::Zero();
		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> rJ 	= Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();

		Matrix<real_t, 6*TP_BODIES, 1> rFe 	= Matrix<real_t, 6*TP_BODIES, 1>::Zero();
		Matrix<real_t, 6*TP_BODIES, 1> rv 	= Matrix<real_t, 6*TP_BODIES, 1>::Zero();

		set_random_J(m, rJ);
		set_random_v(m, rv);
		set_random_Mi(m, rMi);
		set_random_Fe(m, rFe);

		real_t dt = Matrix<real_t, 1, 1>::Random()(0);

		Matrix<real_t, TP_CONSTRAINTS, 1> rrhs = - rJ * (1/dt * rv + rMi * rFe);
		compute_rhs(m, dt);

//		std::cout << "rhs" << std::endl;
//		print_vec(m->rhs, 1, TP_CONSTRAINTS);
//		std::cout << rrhs << std::endl;

		for(int s = 0; s < TP_CONSTRAINTS; ++s)
			TS_ASSERT_DELTA(rrhs(s), _rhs(m, s), 1e-7);

		free(m);
	}
};
