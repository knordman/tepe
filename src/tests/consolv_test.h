/*
 * consolv_test.h
 *
 *  Created on: Jul 23, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cxxtest/TestSuite.h>

// Tests below relies on these values
#define TP_BODIES	3
#define TP_HINGES	1
#define TP_MOTORS	1
#define TP_FEET 	0

#define TP_ERP		0.0

#include <tp/tp-core.h>
#include <debugging/debugging.h>
#include <tp/tp.h>

#include "helpers.h"

#include <Eigen/LU>

class convsolv_test : public CxxTest::TestSuite
{
public:

	/** Tests solving a random linear problem with the constraint solver, see \ref tp-dynamics.
	 *
	 * @ingroup tp-tests
	 */
	void test_solve_system_random()
	{
		struct mem_t *m = stage_memory();

		real_t dt = 1.0;

//		std::cout << std::endl;

		Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES> rJ = Matrix<real_t, TP_CONSTRAINTS, 6*TP_BODIES>::Zero();
		set_random_J(m, rJ);

		Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES> rMi 	= Matrix<real_t, 6*TP_BODIES, 6*TP_BODIES>::Zero();
		set_random_Mi(m, rMi);

		Matrix<real_t, 6*TP_BODIES, 1> rv;
		set_random_v(m, rv);

		Matrix<real_t, 6*TP_BODIES, 1> rFe;
		set_random_Fe(m, rFe);

		Matrix<real_t, TP_CONSTRAINTS, 1> rrhs = -rJ * (1/dt * rv + rMi * rFe);
		Matrix<real_t, TP_CONSTRAINTS, TP_CONSTRAINTS> A = rJ * rMi * rJ.transpose();

		solve_for_lambda(m, dt, 100);

//		print_vec(w->mem->J, 2*TP_SIZE_VEC6, TP_CONSTRAINTS);
//		print_vec(w->mem->Jm, 2, TP_CONSTRAINTS);
//		std::cout << rJ;
//		std::cout << A << std::endl;
//		print_vec(w->mem->d, 1, TP_CONSTRAINTS);

		Matrix<real_t, TP_CONSTRAINTS, 1> rlambda = A.partialPivLu().solve(rrhs);

//		print_vec(w->mem->lambda, 1, TP_CONSTRAINTS);
//		std::cout << rlambda << std::endl;

		for(int s = 0; s < TP_CONSTRAINTS; ++s)
			TS_ASSERT_DELTA(rlambda(s), _lambda(m, s), 1e-2);

		free(m);
	}
};

