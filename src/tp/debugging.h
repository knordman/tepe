/*
 * debugging.h
 *
 *  Created on: Jul 30, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cassert>
#include <iostream>
#include <iomanip>

/** Prints a float array to stdout.
 *
 * Outputs @a rows times @a wrap number of entries, starting from the @a v address.
 * Each entry is multiplied by @a mult before printed.
 *
 * @param	v		Start address.
 * @param	wrap	Length of a row.
 * @param	rows	Number of rows to print.
 * @param	mult	Multiplier for each entry.
 *
 * @ingroup tp-dev
 */
inline void print_vec(real_t *v, int wrap, int rows, real_t mult = 1.0)
{
	std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4);

	int cellw0 = 5;
	int cellw = 15;

	std::cout << std::setw(cellw0) << ' ';
	int k = 0;
	for(k = 0; k < wrap; ++k)
		std::cout << std::setw(cellw) << k;
	std::cout << std::endl;

	k = 0;
	for(int j = 0; j < rows; ++j)
	{
		std::cout << std::setw(cellw0) << j;
		for(int i = 0; i < wrap; ++i, ++k)
		{
			std::cout << std::setw(cellw) << v[k] * mult;
		}
		std::cout << std::endl;
	}
}

/** Prints a index array to stdout.
 *
 * Outputs @a rows times @a wrap number of entries, starting from the @a v address.
 *
 * @param	v		Start address.
 * @param	wrap	Length of a row.
 * @param	rows	Number of rows to print.
 *
 * @ingroup tp-dev
 */
inline void print_vec(index_t *v, int wrap, int rows)
{
	int cellw0 = 5;
	int cellw = 15;

	std::cout << std::setw(cellw0) << ' ';
	int k = 0;
	for(k = 0; k < wrap; ++k)
		std::cout << std::setw(cellw) << k;
	std::cout << std::endl;

	k = 0;
	for(int j = 0; j < rows; ++j)
	{
		std::cout << std::setw(cellw0) << j;
		for(int i = 0; i < wrap; ++i, ++k)
		{
			std::cout << std::setw(cellw) << v[k];
		}
		std::cout << std::endl;
	}
}

/** Prints a 3x3 matrix to stdout.
 *
 * Outputs the label followed by the matrix.
 *
 * @param	m		Matrix to be printed.
 * @param	string	Label for matrix.
 *
 * @ingroup tp-dev
 */
inline void print_mtx33(tp_mtx33 m, const char *string)
{
	std::cout << std::setw(10) << string << "     ";

	for(int j = 0; j < 2; ++j)
	{
		for(int i = 0; i < 3; ++i)
			std::cout << std::setw(9) << m[j*TP_SIZE_VEC3+i] << ' ';

		std::cout << std::endl;
		std::cout << std::setw(15) << ' ';
	}
	for(int i = 0; i < 3; ++i)
		std::cout << std::setw(9) << m[2*TP_SIZE_VEC3+i] << ' ';

	std::cout << std::endl;
}

/** Prints a 3D vector to stdout.
 *
 * Outputs the label followed by the matrix.
 *
 * @param	vec		Vector to be printed.
 * @param	string	Label for vector.
 * @param	norm	Whether to print the L2 norm.
 *
 * @ingroup tp-dev
 */
inline void print_vec3(tp_vec3 vec, const char *string, bool norm = false)
{
	std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4);

	std::cout << string << ": " << vec[0] << ' ' << vec[1] << ' ' << vec[2];
	if(norm)
		std::cout << ", |.| = " << norm2_vec3(vec);

	std::cout << std::endl;
}


/** Checks the correctness of a simulation.
 *
 * Performs some (simple) test to check whether a simulation is setup correctly.
 *
 * @param		m		Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dev
 */
inline void assert_consistency(struct mem_t *m)
{
	assert(TP_BODIES >= 0);
	assert(TP_HINGES <= TP_BODIES - 1);
	assert(TP_MOTORS <= TP_HINGES);
	assert(TP_FEET <= TP_BODIES);

	for(int b = 0; b < TP_BODIES; ++b)
	{
		tp_quatern _quatern;
		get_quatern(quatern(m, b), _quatern);

		tp_mtx33 _Rq;
		quaternion_to_rot_mtx33(_quatern, _Rq);

		for(int row = 0; row < 3; ++row)
			for(int col = 0; col < 3; ++col)
			{
				assert(normalize_quaternion(_quatern));
				assert(abs(_ij(R(m, b), row, col) - _Rq[row*TP_SIZE_VEC3 + col]) < 1e-7);
			}
	}
}
