
#pragma once

/**
 * @name Type Settings
 */
//@{

/**
 * The size of a three dimensional array in number of elements.
 * @ingroup tp-types
 */
#define TP_SIZE_VEC3	4

/**
 * The size of a four dimensional array in number of elements.
 * @ingroup tp-types
 */
#define TP_SIZE_VEC4	4

/**
 * The size of a six dimensional array in number of elements.
 * @ingroup tp-types
 */
#define TP_SIZE_VEC6	6

/**
 * Abstract type for floating point numbers. The floating point number type
 * can either be typedef-ed to @b double or @b float. If using the default
 * type header, the typedef can be set at compile time by the #TP_DEFAULT_SINGLE
 * or #TP_DEFAULT_DOUBLE macro, see \ref tp-usage.
 * @ingroup tp-types
 */
#ifdef TP_DEFAULT_SINGLE
typedef float real_t;
#else
typedef double real_t;
#endif

/**
 * Abstract type for the elements in map arrays. The type for index elements
 * should be of an integer type.
 * @ingroup tp-types
 */
typedef short index_t;

/**
 * The 3D vector type.
 * @ingroup tp-types
 */
typedef real_t tp_vec3[TP_SIZE_VEC3];

/**
 * The quaternion type.
 * @ingroup tp-types
 */
typedef real_t tp_quatern[TP_SIZE_VEC4];

/**
 * The 3x3 matrix type.
 * @ingroup tp-types
 */
typedef real_t tp_mtx33[3*TP_SIZE_VEC3];
//@}

/**
 * @name Math Functions
 */
//@{
#include <cmath>

/**
 * Macro wrapper for square root function to be used.
 * @ingroup tp-types
 */
#define	TP_SQRT(X)		sqrt((X))

/**
 * Macro wrapper for absolute value function to be used (floating point).
 * @ingroup tp-types
 */
#define	TP_ABS(X)		fabs((X))

/**
 * Macro wrapper for arcus tangent function to be used.
 * @ingroup tp-types
 */
#define TP_ATAN2(X, Y)	atan2((X), (Y))

/**
 * Macro wrapper for function computing exponentiation.
 * @ingroup tp-types
 */
#define TP_POW(B, E)	pow((B), (E))

/**
 * Macro wrapper for sine function.
 * @ingroup tp-types
 */
#define TP_SIN(X)		sin((X))

/**
 * Macro wrapper for cosine function.
 * @ingroup tp-types
 */
#define TP_COS(X)		cos((X))

/**
 * Macro wrapper for function computing remainder of division (floating point).
 * @ingroup tp-types
 */
#define TP_FMOD(A, B)	fmod((A), (B))
//@}

/**
 * @name Function Specifiers
 */
//@{
/**
 * Specifier for regular simulator functions. By default the regular simulator
 * functions are not prepended by any specifier.
 * @ingroup tp-types
 */
#define TP_FUNC

/**
 * Specifier for inline simulator functions. The specifier for inline
 * functions are by default @a inline.
 * @ingroup tp-types
 */
#define TP_FUNC_INLINE inline
//@}
