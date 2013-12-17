
#pragma once

/**
 * Memory handle to the memory representing the simulation world. Each memory
 * implementation may define this structure in its own most suitable way. The
 * memory pointed to by this handle is manipulated through the memory access
 * functions.
 *
 * @ingroup tp-mem
 */
struct mem_t *m;

/**
 * @name Memory Access Functions
 *
 * These functions provide a way to write memory implementation independent
 * code, for accessing the simulation data. The
 * typical access pattern is to create a local variable, fetch desired data
 * from memory, do computations and then update the memory. In other words, @a get,
 * @a compute, @a set.
 */
//@{

/**
 * Fetches a vector from memory and stores it in a local vector. The pointer to
 * the memory to be fetched must be obtained by a memory abstraction function.
 * For example, to get the position of the first body use something like:
 * \code{.cpp}
 * tp_vec3 position_body_one_copy;
 * get_vec3(pos(memory_pointer, 1), position_body_one_copy);
 * \endcode
 *
 * @param[in]		vec			Pointer to the memory to be fetched.
 * @param[out]		copy		Vector where to store the fetched memory.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void get_vec3(const real_t *vec, tp_vec3 copy)
{
	copy[0] = _x(vec);
	copy[1] = _y(vec);
	copy[2] = _z(vec);
}

/**
 * Writes a local vector to memory. The pointer to the memory to be overwritten
 * must be obtained by a memory abstraction function.
 * For example, to set the axis of hinge joint 1, use
 * \code{.cpp}
 * tp_vec3 hinge_axis = {0.1, 0.2, 0.3};
 * set_vec3(hinge_axis, haxis(memory_pointer, 1));
 * \endcode
 *
 * @param[in]		new_vec			Vector to be stored to memory.
 * @param[out]		storage			Pointer to the memory that will be set.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void set_vec3(const tp_vec3 new_vec, real_t *storage)
{
	*x(storage) = new_vec[0];
	*y(storage) = new_vec[1];
	*z(storage) = new_vec[2];
}

/**
 * Fetches a quaternion from memory and stores it in a local quaternion. The
 * pointer to the memory to be fetched must be obtained by a memory abstraction
 * function. For example, to get the quaternion of the first body use:
 * \code{.cpp}
 * tp_quaternion quaterion_body_one;
 * get_quatern(quatern(memory_pointer, 1), quaterion_body_one);
 * \endcode
 *
 * @param[in]		quatern			Pointer to the memory to be fetched.
 * @param[out]		copy			Quaternion where to store the fetched memory.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void get_quatern(const real_t *quatern, tp_quatern copy)
{
	copy[0] = _q0(quatern);
	copy[1] = _q1(quatern);
	copy[2] = _q2(quatern);
	copy[3] = _q3(quatern);
}

/**
 * Writes a local quaternion to memory. The pointer to the memory to be overwritten
 * must be obtained by a memory abstraction function.
 * For example, to set the quaternion of the first body, use
 * \code{.cpp}
 * tp_quatern zero_rotation = {1.0, 0.0, 0.0, 0.0};
 * set_quatern(zero_rotation, quatern(memory_pointer, 1));
 * \endcode
 *
 * @param[in]		nquatern			Quaternion to be stored to memory.
 * @param[out]		quatern				Pointer to the memory that will be set.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void set_quatern(const tp_quatern nquatern, real_t *quatern)
{
	*q0(quatern) = nquatern[0];
	*q1(quatern) = nquatern[1];
	*q2(quatern) = nquatern[2];
	*q3(quatern) = nquatern[3];
}

/**
 * Fetches a 3x3 matrix from memory and stores it in a local 3x3 matrix. The
 * pointer to the memory to be fetched must be obtained by a memory abstraction
 * function. For example, to get the rotation matrix of the first body use:
 * \code{.cpp}
 * tp_mtx33 rotation_body_one;
 * get_mtx33(R(memory_pointer, 1), rotation_body_one);
 * \endcode
 *
 * @param[in]		mtx				Pointer to the memory to be fetched.
 * @param[out]		copy			3x3 matrix where to store the fetched memory.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void get_mtx33(const real_t *mtx, tp_mtx33 copy)
{
	copy[0] = 					_ij(mtx, 0, 0);
	copy[1] =		 			_ij(mtx, 0, 1);
	copy[2] = 					_ij(mtx, 0, 2);

	copy[TP_SIZE_VEC3] = 		_ij(mtx, 1, 0);
	copy[TP_SIZE_VEC3+1] = 		_ij(mtx, 1, 1);
	copy[TP_SIZE_VEC3+2] = 		_ij(mtx, 1, 2);

	copy[2*TP_SIZE_VEC3] = 		_ij(mtx, 2, 0);
	copy[2*TP_SIZE_VEC3+1] = 	_ij(mtx, 2, 1);
	copy[2*TP_SIZE_VEC3+2] = 	_ij(mtx, 2, 2);
}

/**
 * Writes a local 3x3 matrix to memory. The pointer to the memory to be overwritten
 * must be obtained by a memory abstraction function.
 * For example, to set the inverse inertia tensor of the first body, use
 * \code{.cpp}
 * tp_mtx33 body_one_inertia = ... // Inertia definition
 * set_mtx33(body_one_inertia, Ibi(memory_pointer, 1));
 * \endcode
 *
 * @param[in]		new_mtx			3x3 matrix to be stored to memory.
 * @param[out]		storage			Pointer to the memory that will be set.
 *
 * @ingroup tp-mem
 */
TP_FUNC_INLINE
void set_mtx33(const tp_mtx33 new_mtx, real_t *storage)
{
	*ij(storage, 0, 0) = new_mtx[0];
	*ij(storage, 0, 1) = new_mtx[1];
	*ij(storage, 0, 2) = new_mtx[2];

	*ij(storage, 1, 0) = new_mtx[TP_SIZE_VEC3];
	*ij(storage, 1, 1) = new_mtx[TP_SIZE_VEC3+1];
	*ij(storage, 1, 2) = new_mtx[TP_SIZE_VEC3+2];

	*ij(storage, 2, 0) = new_mtx[2*TP_SIZE_VEC3];
	*ij(storage, 2, 1) = new_mtx[2*TP_SIZE_VEC3+1];
	*ij(storage, 2, 2) = new_mtx[2*TP_SIZE_VEC3+2];
}
//@}

/**
 * @name Memory Access Implementation Unique Functions
 *
 * These functions need to be implemented by each memory abstraction. Even
 * though the memory access functions provide the means to operate on the
 * memory, it is sometimes useful to use these lower level functions directly.
 * As an example, setting the x-component of the external force of the first
 * body can be done by:
 * \code{.c}
 * *x(tFe(memory_pointer, 1)) = 1.0;
 * \endcode
 * See below for an explanation of @a x and @a tFe. The naming convention is
 * that functions returning a value are prepended by an underscore. Functions
 * that return the address to this value have the same name except that they
 * lack the underscore.
 */
//@{

/**
 * Zero initializes a simulation world.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE void zero_memory(struct mem_t *m);

/**
 * Returns a pointer to the first element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @return Pointer to the first element of the 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * x(real_t *vec3);

/**
 * Returns the first element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @return First element of the 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _x(const real_t *vec3);

/**
 * Returns a pointer to the second element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * y(real_t *vec3);

/**
 * Returns the second element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @return Pointer to the second element of the 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _y(const real_t *vec3);

/**
 * Returns a pointer to the third element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @return Second element of the 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * z(real_t *vec3);

/**
 * Returns the third element in a 3D vector.
 *
 * @param		vec3		Memory pointer for a 3D vector.
 * @return Third element of the 3D vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _z(const real_t *vec3);

/**
 * Returns a pointer to the first element of a quaternion.
 *
 * @param		quatern		Memory pointer for a quaternion.
 * @return Pointer to the first element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * q0(real_t *quatern);

/**
 * Returns the first element in a quaternion.
 *
 * @param		quatern		Memory pointer for a 3D vector.
 * @return First element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _q0(const real_t *quatern);

/**
 * Returns a pointer to the second element of a quaternion.
 *
 * @param		quatern		Memory pointer for a quaternion.
 * @return Pointer to the second element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * q1(real_t *quatern);

/**
 * Returns the second element in a quaternion.
 *
 * @param		quatern		Memory pointer for a 3D vector.
 * @return Second element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _q1(const real_t *quatern);

/**
 * Returns a pointer to the third element of a quaternion.
 *
 * @param		quatern		Memory pointer for a quaternion.
 * @return Pointer to the third element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * q2(real_t *quatern);

/**
 * Returns the third element in a quaternion.
 *
 * @param		quatern		Memory pointer for a 3D vector.
 * @return Third element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _q2(const real_t *quatern);

/**
 * Returns a pointer to the fourth element of a quaternion.
 *
 * @param		quatern		Memory pointer for a quaternion.
 * @return Pointer to the fourth element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * q3(real_t *quatern);

/**
 * Returns the fourth element in a quaternion.
 *
 * @param		quatern		Memory pointer for a 3D vector.
 * @return Fourth element of the quaternion.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _q3(const real_t *quatern);

/**
 * Returns a pointer to an element of a 3x3 matrix.
 *
 * @param		mtx33		Memory pointer for a 3x3 matrix.
 * @param		row			Row index for element, zero based.
 * @param		col			Column index for element, zero based.
 * @return Pointer to the matrix element.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * ij(real_t *mtx33, index_t row, index_t col);

/**
 * Returns an element of a 3x3 matrix.
 *
 * @param		mtx33		Memory pointer for a 3x3 matrix.
 * @param		row			Row index for element, zero based.
 * @param		col			Column index for element, zero based.
 * @return Matrix entry.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _ij(const real_t *mtx33, index_t row, index_t col);

/**
 * Returns memory pointer to hinge axis.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Number of hinge whose axis pointer is to be returned.
 * @returns Vector pointer for hinge axis.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * haxis(struct mem_t *m, index_t hinge_num);

/**
 * Returns memory pointer to hinge axis, as seen from either body. In theory
 * this axis is the same axis for both bodies. However, numerical
 * drift and solver inaccuracy cause some difference to occur.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Number of hinge whose axis pointer is to be returned.
 * @param			body_index	0 or 1, whether to return axis of first or second body.
 * @returns Vector pointer for hinge axis for either body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * haxis_num(struct mem_t *m, index_t hinge_num, index_t body_index);

/**
 * Returns memory pointer of first tangent vector to hinge axis.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Number of hinge whose tangent is to be returned.
 * @returns Vector pointer for first tangent to hinge axis.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * ht0(struct mem_t *m, index_t hinge_num);

/**
 * Returns memory pointer of second tangent vector to hinge axis. The first and
 * second tangent form the tangent plane to the axis. In other words the second
 * tangent is orthogonal to the first tangent.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Number of hinge whose tangent is to be returned.
 * @returns Vector pointer for first tangent to hinge axis.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * ht1(struct mem_t *m, index_t hinge_num);

/**
 * Returns memory pointer to the hinge anchors. Two hinge anchors may be returned,
 * one for each body that the hinge connects. The hinge anchor is a vector from
 * the center of gravity to a point, such that the hinge constraints the motion of
 * the two bodies in such a way that the hinge anchors are constant.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Number of hinge whose anchor is to be returned.
 * @param			body_index	0 or 1, whether to return anchor of first or second body.
 * @returns Vector pointer for hinge anchor.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * hanchor(struct mem_t *m, index_t hinge_num, index_t body_index);

/**
 * Returns memory pointer to position vector of body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Vector pointer for position vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * pos(struct mem_t *m, index_t body);

/**
 * Returns memory pointer to translational velocity vector of body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Vector pointer for velocity vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * vel(struct mem_t *m, index_t body);

/**
 * Returns memory pointer to angular velocity vector of body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Vector pointer for velocity vector.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * omega(struct mem_t *m, index_t body);

/**
 * Returns memory pointer to quaternion (represents the rotation) of body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Quaternion for body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * quatern(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the inverse mass of a body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Pointer to inverse mass.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * mi(struct mem_t *m, index_t body);

/**
 * Returns the inverse mass of a body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Inverse mass.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _mi(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the rotation matrix of a body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Pointer to the rotation matrix of a body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * R(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the inverse inertia tensior of a body. The inverse
 * tensor is given in body coordinates.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, TP_BODIES-1].
 * @returns Pointer to the inverse inertia tensor of a body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * Ibi(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the translation external force acting on a body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to translational external force acting on a body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * tFe(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the angular external force (external torque)
 * acting on a body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to external torque acting on a body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * aFe(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the constraint body mapping of either first or second body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Pointer to translational external force acting on a body.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t * Jm(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns the mapped bodies of a constraint. For constraints that contain only one
 * body, the function will return -1 when requesting mapping for either first or
 * second body.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Body that is mapped to the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t _Jm(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns a memory pointer to the translational part of a Jacobian row.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Pointer to translational part of a Jacobian row.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * tJ(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns a memory pointer to the angular part of a Jacobian row.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Pointer to angular part of a Jacobian row.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * aJ(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns a memory pointer to the translational part of the \f$B\f$ variable.
 * \see compute_B.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Pointer to translational part of the \f$B\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * tB(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns a memory pointer to the angular part of the \f$B\f$ variable.
 * \see compute_B.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @param 			body_index	0 or 1, first or second body.
 * @returns Pointer to angular part of the \f$B\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * aB(struct mem_t *m, index_t constraint, index_t body_index);

/**
 * Returns a memory pointer to the translational part of the \f$a\f$ variable.
 * \see compute_a.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to translational part of the \f$a\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * ta(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the angular part of the \f$a\f$ variable.
 * \see compute_a.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to angular part of the \f$a\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * aa(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the Lagrange multiplier of a constraint.
 * \see solve_for_lambda.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Pointer to the Lagrange multiplier of the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * lambda(struct mem_t *m, index_t constraint);

/**
 * Returns the Lagrange multiplier of a constraint.
 * \see solve_for_lambda.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Lagrange multiplier of the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _lambda(struct mem_t *m, index_t constraint);

/**
 * Returns a memory pointer to the minimum allowed value for the Lagrange
 * multiplier of a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Pointer to the minimum allowed Lagrange multiplier for constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * lambda_min(struct mem_t *m, index_t constraint);

/**
 * Returns the minimum allowed value for the Lagrange multiplier of a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Minimum allowed Lagrange multiplier for the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _lambda_min(struct mem_t *m, index_t constraint);

/**
 * Returns a memory pointer to the maximum allowed value for the Lagrange
 * multiplier of a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Pointer to the maximum allowed Lagrange multiplier for constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * lambda_max(struct mem_t *m, index_t constraint);

/**
 * Returns the maximum allowed value for the Lagrange multiplier of a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Maximum allowed Lagrange multiplier for the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _lambda_max(struct mem_t *m, index_t constraint);

/**
 * Returns a memory pointer to the entry in the \f$d\f$ variable connected to
 * a constraint. \see compute_d.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Pointer to constraint entry in \f$d\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * d(struct mem_t *m, index_t constraint);

/**
 * Returns the entry in the \f$d\f$ variable connected to a constraint.
 * \see compute_d.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Constraint entry in \f$d\f$ variable.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _d(struct mem_t *m, index_t constraint);

/**
 * Returns a memory pointer to right hand side vector entry for a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Pointer to right hand side entry for the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * rhs(struct mem_t *m, index_t constraint);

/**
 * Returns the right hand side vector entry for a constraint.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			constraint	Constraint to query, in interval [0, #TP_CONSTRAINTS-1].
 * @returns Right hand side entry for the constraint.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _rhs(struct mem_t *m, index_t constraint);

/**
 * Returns a memory pointer to desired angular velocity of a motor.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			motor		Motor to query, in interval [0, #TP_MOTORS-1].
 * @returns Pointer to desired speed of the motor.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * mds(struct mem_t *m, index_t motor);

/**
 * Returns the desired angular velocity of a motor.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			motor		Motor to query, in interval [0, #TP_MOTORS-1].
 * @returns Desired speed of the motor.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t _mds(struct mem_t *m, index_t motor);

/**
 * Returns a memory pointer to the motor map entry connecting a motor to a hinge.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			motor		Motor to query, in interval [0, #TP_MOTORS-1].
 * @returns Pointer to the motor map entry connecting the motor to a hinge.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t * mm(struct mem_t *m, index_t motor);

/**
 * Returns the motor map entry connecting a motor to a hinge.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			motor		Motor to query, in interval [0, #TP_MOTORS-1].
 * @returns Motor map entry connecting the motor to a hinge.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t _mm(struct mem_t *m, index_t motor);

/**
 * Returns a memory pointer to the initial rotation quaternion for a hinge.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			hinge_num	Hinge to query, in interval [0, #TP_HINGES-1].
 * @returns Pointer to the initial rotation quaternion for the hinge.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * iniquatern(struct mem_t *m, index_t hinge_num);

#ifdef TP_DEBUG

/**
 * Returns a memory pointer to the translational constraint force on a body.
 * Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to the translational constraint force.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * tFc(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the angular constraint force on a body.
 * Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			body		Body to query, in interval [0, #TP_BODIES-1].
 * @returns Pointer to the angular constraint force.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * aFc(struct mem_t *m, index_t body);

/**
 * Returns a memory pointer to the contact point of a contact.
 * Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			contact		Contact to query. Maximum contact index is 3 x #TP_FEET.
 * @returns Pointer to 3D vector for the contact point.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * cpo(struct mem_t *m, index_t contact);

/**
 * Returns a memory pointer to the contact normal of a contact.
 * Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			contact		Contact to query. Maximum number of contacts are 3 x #TP_FEET.
 * @returns Pointer to 3D vector for the contact normal.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * cno(struct mem_t *m, index_t contact);

/**
 * Returns a memory pointer to a vector in the contact plane orthogonal to the
 * contact normal. Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			foot		Foot to query, in interval [0, #TP_FEET-1].
 * @returns Pointer to 3D vector in the contact plane.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * cpl0(struct mem_t *m, index_t foot);

/**
 * Returns a memory pointer to a vector in the contact plane orthogonal to the
 * contact normal. Also orthogonal to the vector returned by cpl0. Only available
 * when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			foot		Foot to query, in interval [0, #TP_FEET-1].
 * @returns Pointer to 3D vector in the contact plane.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE real_t * cpl1(struct mem_t *m, index_t foot);

/**
 * Returns a memory pointer to an index holding the body connected to a
 * contact. Only available when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			foot		Foot to query, in interval [0, #TP_FEET-1].
 * @returns Pointer to index of body that is in contact.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t * cbdy(struct mem_t *m, index_t foot);

/**
 * Returns the index holding the body connected to a contact. Only available
 * when #TP_DEBUG is defined.
 *
 * @param			m			Pointer to the memory representing the simulation world.
 * @param			foot		Foot to query, in interval [0, #TP_FEET-1].
 * @returns Body that is in contact.
 * @ingroup tp-mem
 */
TP_FUNC_INLINE index_t _cbdy(struct mem_t *m, index_t foot);
#endif
//@}
