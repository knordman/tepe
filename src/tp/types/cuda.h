/*
 * types.h
 *
 *  Created on: Jul 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#pragma once


// Type information -------------------------------------------------

#define TP_SIZE_VEC3	4
#define TP_SIZE_VEC4	4
#define TP_SIZE_VEC6	6

typedef float real_t;
typedef short index_t;
typedef real_t tp_vec3[TP_SIZE_VEC3];
typedef real_t tp_quatern[TP_SIZE_VEC4];
typedef real_t tp_mtx33[3*TP_SIZE_VEC3];

// Math function information ----------------------------------------

#define	TP_SQRT(X)		sqrtf((X))
#define	TP_ABS(X)		fabsf((X))
#define TP_ATAN2(X, Y)	atan2f((X), (Y))
#define TP_POW(B, E)	powf((B), (E))
#define TP_SIN(X)		sinf((X))
#define TP_COS(X)		cosf((X))
#define TP_FMOD(A, B)	fmodf((A), (B))


// Function specification -------------------------------------------

#define TP_FUNC __device__

#define TP_FUNC_INLINE __host__ __device__ __forceinline__

