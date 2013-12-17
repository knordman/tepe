
#pragma once


/** Adds two 3D vectors.
 *
 * The input 3D vectors @a a and @a b are added together after @a b has been
 * multiplied with @a mult.
 *
 * @param[out]	result	The vector to store the result in.
 * @param[in]	a		Input vector a.
 * @param[in]	b		Input vector b.
 * @param		mult	Multiplier for b.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void add_vec3(tp_vec3 result, const tp_vec3 a, const tp_vec3 b, real_t mult)
{
	result[0] = a[0] + mult*b[0];
	result[1] = a[1] + mult*b[1];
	result[2] = a[2] + mult*b[2];
}

/** Adds two 3D vectors storing the result into the first parameter.
 *
 * The input vectors @a a and @a b are added together after @a b has been
 * multiplied with @a mult. The result overwrites @a a.
 *
 * @param[in,out]	a		Input vector a.
 * @param[in]		b		Input vector b.
 * @param			mult	Multiplier for b.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void add_to_vec3(tp_vec3 a, const tp_vec3 b, real_t mult)
{
	a[0] += mult*b[0];
	a[1] += mult*b[1];
	a[2] += mult*b[2];
}

/** Computes the dot product of two 3D vectors.
 *
 * @param[in]		a		Input vector a.
 * @param[in]		b		Input vector b.
 * @return the vector dot product of @c a and @a b.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
real_t dot_vec3(const tp_vec3 a, const tp_vec3 b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** Scales a 3D vector.
 *
 * @param[out]		result		The vector to store the result in.
 * @param[in]		vec			Input vector.
 * @param			mult		Scaling factor.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void scale_vec3(tp_vec3 result, const tp_vec3 vec, real_t mult)
{
	result[0] = vec[0]*mult;
	result[1] = vec[1]*mult;
	result[2] = vec[2]*mult;
}

/** Scales a 3D vector and overwrites it.
 *
 * The input vector is overwritten by the scaled vector.
 *
 * @param[in,out]	vec			Input vector.
 * @param			mult		Scaling factor.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void scale_to_vec3(tp_vec3 vec, real_t mult)
{
	vec[0] *= mult;
	vec[1] *= mult;
	vec[2] *= mult;
}

/** Computes the square of the L2 norm of a 3D vector.
 *
 *
 * @param[in,out]	vec			Input vector.
 * @return square of L2 norm, same as length of vector squared
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
real_t norm22_vec3(const tp_vec3 vec)
{
	return dot_vec3(vec, vec);
}

/** Computes the L2 norm of a 3D vector.
 *
 *
 * @param[in,out]	vec			Input vector.
 * @return L2 norm, same as length of vector
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
real_t norm2_vec3(const tp_vec3 vec)
{
	return TP_SQRT(dot_vec3(vec, vec));
}

/** Normalizes a 3D vector.
 *
 * Tries to normalize the vector. If the length of the vector is too small, the
 * input vector is left unchanged and @b false is returned.
 *
 * @param[in,out]	vec			Input vector.
 * @return @b true if normalization successful, @b false if unsuccessful.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
bool normalize_vec3(tp_vec3 vec)
{
	real_t len2 = norm22_vec3(vec);
	if(len2 < TP_REAL(1e-7)) return false;
	real_t len = TP_SQRT(len2);
	vec[0] /= len;
	vec[1] /= len;
	vec[2] /= len;
	return true;
}

/** Computes the cross product of two 3D vectors.
 *
 * @param[out]		result		The vector to store the result in.
 * @param[in]		a			Input vector a.
 * @param[in]		b			Input vector b.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void cross_vec3(tp_vec3 result, const tp_vec3 a, const tp_vec3 b)
{
	result[0] = a[1]*b[2] - b[1]*a[2];
	result[1] = -a[0]*b[2] + b[0]*a[2];
	result[2] = a[0]*b[1] - b[0]*a[1];
}

/** Computes the cross product of two 3D vectors storing the result into the
 * first parameter.
 *
 * @param[in]		a			Input vector a.
 * @param[in]		b			Input vector b.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void cross_to_vec3(tp_vec3 a, const tp_vec3 b)
{
	tp_vec3 result;
	result[0] = a[1]*b[2] - b[1]*a[2];
	result[1] = -a[0]*b[2] + b[0]*a[2];
	result[2] = a[0]*b[1] - b[0]*a[1];
	a[0] = result[0];
	a[1] = result[1];
	a[2] = result[2];
}

/** Sets a 3x3 matrix to the identity matrix.
 *
 * @param[in, out]		mtx			Input matrix.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void set_to_ident_mtx33(tp_mtx33 mtx)
{
	mtx[0*TP_SIZE_VEC3+0] = TP_REAL(1.0);
	mtx[0*TP_SIZE_VEC3+1] = TP_REAL(0.0);
	mtx[0*TP_SIZE_VEC3+2] = TP_REAL(0.0);

	mtx[1*TP_SIZE_VEC3+0] = TP_REAL(0.0);
	mtx[1*TP_SIZE_VEC3+1] = TP_REAL(1.0);
	mtx[1*TP_SIZE_VEC3+2] = TP_REAL(0.0);

	mtx[2*TP_SIZE_VEC3+0] = TP_REAL(0.0);
	mtx[2*TP_SIZE_VEC3+1] = TP_REAL(0.0);
	mtx[2*TP_SIZE_VEC3+2] = TP_REAL(1.0);
}

/** Multiplies a 3x1 vector by a 3x3 matrix.
 *
 * @param[out]		result		The vector to store the result in.
 * @param[in]		mtx			Input matrix.
 * @param[in]		vec			Input vector.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_mtx33_vec3(tp_vec3 result, const tp_mtx33 mtx, const tp_vec3 vec)
{
	result[0] = vec[0]*mtx[0*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[0*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[0*TP_SIZE_VEC3+2];

	result[1] = vec[0]*mtx[1*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[1*TP_SIZE_VEC3+2];

	result[2] = vec[0]*mtx[2*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[2*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+2];
}

/** Multiplies a 3x1 vector by a 3x3 matrix storing the result in the input vector.
 *
 * @param[in]		mtx			Input matrix.
 * @param[in]		vec			Input vector.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_to_mtx33_vec3(const tp_mtx33 mtx, tp_vec3 vec)
{
	real_t tmp1, tmp2;
	tmp1 = vec[0]*mtx[0*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[0*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[0*TP_SIZE_VEC3+2];

	tmp2 = vec[0]*mtx[1*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[1*TP_SIZE_VEC3+2];

	vec[2] = vec[0]*mtx[2*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[2*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+2];
	vec[0] = tmp1;
	vec[1] = tmp2;
}


/** Transposes a 3x3 matrix.
 *
 * @param[out]		mtxT			The matrix to store the transpose in.
 * @param[in]		mtx				Input matrix.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void transpose_mtx33(tp_mtx33 mtxT, const tp_mtx33 mtx)
{
	mtxT[0*TP_SIZE_VEC3+0] = mtx[0*TP_SIZE_VEC3+0];
	mtxT[0*TP_SIZE_VEC3+1] = mtx[1*TP_SIZE_VEC3+0];
	mtxT[0*TP_SIZE_VEC3+2] = mtx[2*TP_SIZE_VEC3+0];

	mtxT[1*TP_SIZE_VEC3+0] = mtx[0*TP_SIZE_VEC3+1];
	mtxT[1*TP_SIZE_VEC3+1] = mtx[1*TP_SIZE_VEC3+1];
	mtxT[1*TP_SIZE_VEC3+2] = mtx[2*TP_SIZE_VEC3+1];

	mtxT[2*TP_SIZE_VEC3+0] = mtx[0*TP_SIZE_VEC3+2];
	mtxT[2*TP_SIZE_VEC3+2] = mtx[2*TP_SIZE_VEC3+2];
	mtxT[2*TP_SIZE_VEC3+1] = mtx[1*TP_SIZE_VEC3+2];
}

/** Transposes a 3x3 matrix and overwrites the original matrix.
 *
 * @param[in,out]		R			Input matrix.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void transpose_to_mtx33(tp_mtx33 R)
{
	real_t swap;
	swap = R[1*TP_SIZE_VEC3+0];
	R[1*TP_SIZE_VEC3+0] = R[0*TP_SIZE_VEC3+1];
	R[0*TP_SIZE_VEC3+1] = swap;

	swap = R[0*TP_SIZE_VEC3+2];
	R[0*TP_SIZE_VEC3+2] = R[2*TP_SIZE_VEC3+0];
	R[2*TP_SIZE_VEC3+0] = swap;

	swap = R[1*TP_SIZE_VEC3+2];
	R[1*TP_SIZE_VEC3+2] = R[2*TP_SIZE_VEC3+1];
	R[2*TP_SIZE_VEC3+1] = swap;
}

/** Multiplies a 3x3 matrix by a 3x3 matrix.
 *
 * Multiplies matrix @a A by matrix @a B, result = AB.
 *
 * @param[out]		result			The matrix to store the result in.
 * @param[in]		A				Input matrix A.
 * @param[in]		B				Input matrix B.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_mtx33_mtx33(tp_mtx33 result, const tp_mtx33 A, const tp_mtx33 B)
{
	tp_vec3 col;
	for(int i = 0; i < 3; ++i)
	{
		col[0] = B[0*TP_SIZE_VEC3+i];
		col[1] = B[1*TP_SIZE_VEC3+i];
		col[2] = B[2*TP_SIZE_VEC3+i];

		mult_to_mtx33_vec3(A, col);
		result[0*TP_SIZE_VEC3+i] = col[0];
		result[1*TP_SIZE_VEC3+i] = col[1];
		result[2*TP_SIZE_VEC3+i] = col[2];
	}
}

/** Multiplies a 3x3 matrix by the transpose of a 3x3 matrix.
 *
 * Multiplies matrix @a A by the transpose of matrix @a B, result = <em>AB</em>^T
 *
 * @param[out]		result			The matrix to store the result in.
 * @param[in]		A				Input matrix A.
 * @param[in]		B				Input matrix B.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_mtx33_mtx33T(tp_mtx33 result, const tp_mtx33 A, const tp_mtx33 B)
{
	tp_vec3 col, colres;
	for(int i = 0; i < 3; ++i)
	{
		col[0] = B[0+TP_SIZE_VEC3*i];
		col[1] = B[1+TP_SIZE_VEC3*i];
		col[2] = B[2+TP_SIZE_VEC3*i];

		mult_mtx33_vec3(colres, A, col);
		result[0*TP_SIZE_VEC3+i] = colres[0];
		result[1*TP_SIZE_VEC3+i] = colres[1];
		result[2*TP_SIZE_VEC3+i] = colres[2];
	}
}

/** Multiplies a 3x1 vector by the transpose of a 3x3 matrix.
 *
 * Multiplies vector @a vec by the transpose of matrix @a mtx,
 * result = <em>mtx</em>^T<em>vec</em>
 *
 * @param[out]		result			The matrix to store the result in.
 * @param[in]		mtx				Input matrix.
 * @param[in]		vec				Input vector.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_mtx33T_vec3(tp_mtx33 result, const tp_mtx33 mtx, const tp_vec3 vec)
{
	result[0] = vec[0]*mtx[0*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+0]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+0];

	result[1] = vec[0]*mtx[0*TP_SIZE_VEC3+1]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+1];

	result[2] = vec[0]*mtx[0*TP_SIZE_VEC3+2]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+2]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+2];
}

/** Multiplies a 3x1 vector by the transpose of a 3x3 matrix storing the
 * result in the input vector.
 *
 * Multiplies vector @a vec by the transpose of matrix @a mtx,
 * result = <em>mtx</em>^T<em>vec</em>. Overwrites the input vector.
 *
 * @param[in]		mtx				Input matrix.
 * @param[in]		vec				Input vector.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_to_mtx33T_vec3(const tp_mtx33 mtx, tp_vec3 vec)
{
	real_t tmp1, tmp2;
	tmp1 = vec[0]*mtx[0*TP_SIZE_VEC3+0]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+0]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+0];

	tmp2 = vec[0]*mtx[0*TP_SIZE_VEC3+1]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+1]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+1];

	vec[2] = vec[0]*mtx[0*TP_SIZE_VEC3+2]
	            + vec[1]*mtx[1*TP_SIZE_VEC3+2]
	            + vec[2]*mtx[2*TP_SIZE_VEC3+2];

	vec[0] = tmp1;
	vec[1] = tmp2;
}

/** Converts a quaternion to a 3x3 rotation matrix.
 *
 * @param[in]		q			Quaternion to convert.
 * @param[out]		R			Matrix to store the result in.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void quaternion_to_rot_mtx33(const tp_quatern q, tp_mtx33 R)
{
	R[0*TP_SIZE_VEC3+0] = TP_REAL(1.0) - TP_REAL(2.0)*q[2]*q[2] - TP_REAL(2.0)*q[3]*q[3];
	R[0*TP_SIZE_VEC3+1] = TP_REAL(2.0)*q[1]*q[2] - TP_REAL(2.0)*q[0]*q[3];
	R[0*TP_SIZE_VEC3+2] = TP_REAL(2.0)*q[1]*q[3] + TP_REAL(2.0)*q[0]*q[2];

	R[1*TP_SIZE_VEC3+0] = TP_REAL(2.0)*q[1]*q[2] + TP_REAL(2.0)*q[0]*q[3];
	R[1*TP_SIZE_VEC3+1] = TP_REAL(1.0) - TP_REAL(2.0)*q[1]*q[1] - TP_REAL(2.0)*q[3]*q[3];
	R[1*TP_SIZE_VEC3+2] = TP_REAL(2.0)*q[2]*q[3] - TP_REAL(2.0)*q[0]*q[1];

	R[2*TP_SIZE_VEC3+0] = TP_REAL(2.0)*q[1]*q[3] - TP_REAL(2.0)*q[0]*q[2];
	R[2*TP_SIZE_VEC3+1] = TP_REAL(2.0)*q[2]*q[3] + TP_REAL(2.0)*q[0]*q[1];
	R[2*TP_SIZE_VEC3+2] = TP_REAL(1.0) - TP_REAL(2.0)*q[1]*q[1] - TP_REAL(2.0)*q[2]*q[2];
}

/** Computes a special product between a quaternion and a 3D vector.
 *
 * Computes the product between the quaternion with 0 as scalar part and
 * @a omega as it vector part and a quaternion. Useful when computing the
 * time derivate of a quaternion from an angular velocity.
 *
 * @param[out]		result			The quaternion to store the result in.
 * @param[in]		omega			Input vector (angular velocity).
 * @param[in]		q				Input quaternion.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_omega_quatern(tp_quatern result, const tp_vec3 omega, const tp_quatern q)
{
	tp_vec3 tmp;
	result[0] = -dot_vec3(omega, q+1);
	cross_vec3(tmp, omega, q+1);
	scale_vec3(result+1, omega, q[0]);
	add_to_vec3(result+1, tmp, TP_REAL(1.0));
}

/** Multiplies one quaternion with another quaternion.
 *
 * Computes the @a qa times @a qb quaternion product.
 *
 * @param[out]		result			The quaternion to store the result in.
 * @param[in]		qa				Input quaternion @a qa.
 * @param[in]		qb				Input quaternion @a qb.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
void mult_quatern_quatern(tp_quatern result, const tp_quatern qa, const tp_quatern qb)
{
	result[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
	result[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
	result[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
	result[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
}

/** Normalizes a quaternion.
 *
 * Tries to normalize the quaternion. If the length of the quaternion is too
 * small, the input quaternion is left unchanged and @b false is returned.
 *
 * @param[out,in]		q			Input quaternion.
 * @return @b true if normalization successful, @b false if unsuccessful.
 *
 * @ingroup tp-alglin
 */
TP_FUNC_INLINE
bool normalize_quaternion(tp_quatern q)
{
	real_t magnitude2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	if(magnitude2 < TP_REAL(1e-7)) return false;
	real_t magnitude = TP_SQRT(magnitude2);
	q[0] = q[0] / magnitude;
	q[1] = q[1] / magnitude;
	q[2] = q[2] / magnitude;
	q[3] = q[3] / magnitude;
	return true;
}
