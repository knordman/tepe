/*
 * alloc.h
 *
 *  Created on: Jul 24, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once


// The memory layout
struct mem_t
{
	real_t q[(TP_BODIES)*(TP_SIZE_VEC3+TP_SIZE_VEC4)];		// Generalized position variable, pos + quatern
	real_t v[(TP_BODIES)*TP_SIZE_VEC6];						// Generalized velocity variable, vel + omega
	real_t mi[(TP_BODIES)];									// Inverse mass
	real_t Ibi[(TP_BODIES)*3*TP_SIZE_VEC3];					// Inverse inertia matrix
	real_t R[(TP_BODIES)*3*TP_SIZE_VEC3]; 					// Convenience matrix

	real_t Fe[(TP_BODIES)*TP_SIZE_VEC6];					// External force

	real_t J[2*TP_SIZE_VEC6*TP_CONSTRAINTS];				// Constraint Jacobian

	real_t lambda[TP_CONSTRAINTS];							// F_c = J^{T}\lambda
	real_t lambda_min[TP_CONSTRAINTS];						// min
	real_t lambda_max[TP_CONSTRAINTS];						// max

	index_t mm[(TP_MOTORS)];								// Mapping motors->hinges
	real_t mdspeed[(TP_MOTORS)];							// Desired speed for motors
	real_t iniq[(TP_HINGES)*TP_SIZE_VEC4];					// Quaternions for initial rotations

	index_t Jm[2*TP_CONSTRAINTS];							// Mapping->bodies, sparse Jacobian
	real_t B[2*TP_SIZE_VEC6*TP_CONSTRAINTS];				// M^{-1}J^{T}, for solving
	real_t a[(TP_BODIES)*TP_SIZE_VEC6];						// B\lambda, for solving
	real_t d[TP_CONSTRAINTS];								// diag(JB), for solving
	real_t rhs[TP_CONSTRAINTS];								// Right hand side, for solving

	real_t haxes[(TP_HINGES)*2*TP_SIZE_VEC6];				// Hinge axis 1+2, tangent base 1
	real_t hanchors[(TP_HINGES)*TP_SIZE_VEC6];				// Hinge anchors ( -''- )

#ifdef TP_DEBUG
	real_t Fc[(TP_BODIES)*TP_SIZE_VEC6];					// Constraint force
	real_t cinfo[3*(TP_FEET)*TP_SIZE_VEC6];					// Contact points + contact normals
	real_t cplane[(TP_FEET)*TP_SIZE_VEC6];					// Contact plane (axes where slip is eliminated)
	index_t cbody[(TP_FEET)];								// Body indexes connected to feet
#endif
};

TP_FUNC_INLINE
void zero_memory(struct mem_t *mem)
{
	for(size_t i = 0; i < (TP_BODIES)*(TP_SIZE_VEC3+TP_SIZE_VEC4); ++i) mem->q[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES)*TP_SIZE_VEC6; ++i) mem->v[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES); ++i) mem->mi[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES)*3*TP_SIZE_VEC3; ++i) mem->Ibi[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES)*3*TP_SIZE_VEC3; ++i) mem->R[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES)*TP_SIZE_VEC6; ++i) mem->Fe[i] = TP_REAL(0.0);
	for(size_t i = 0; i < 2*TP_SIZE_VEC6*TP_CONSTRAINTS; ++i) mem->J[i] = TP_REAL(0.0);

	for(size_t i = 0; i < TP_CONSTRAINTS; ++i) mem->lambda[i] = TP_REAL(0.0);
	for(size_t i = 0; i < TP_CONSTRAINTS; ++i) mem->lambda_min[i] = TP_REAL(-1048576.0);
	for(size_t i = 0; i < TP_CONSTRAINTS; ++i) mem->lambda_max[i] = TP_REAL(1048576.0);

	for(size_t i = 0; i < 2*TP_CONSTRAINTS; ++i) mem->Jm[i] = 0;
	for(size_t i = 0; i < 2*TP_SIZE_VEC6*TP_CONSTRAINTS; ++i) mem->B[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_BODIES)*TP_SIZE_VEC6; ++i) mem->a[i] = TP_REAL(0.0);
	for(size_t i = 0; i < TP_CONSTRAINTS; ++i) mem->d[i] = TP_REAL(0.0);
	for(size_t i = 0; i < TP_CONSTRAINTS; ++i) mem->rhs[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_HINGES)*TP_SIZE_VEC6; ++i) mem->hanchors[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_HINGES)*2*TP_SIZE_VEC6; ++i) mem->haxes[i] = TP_REAL(0.0);

	for(size_t i = 0; i < (TP_MOTORS); ++i) mem->mm[i] = 0;
	for(size_t i = 0; i < (TP_MOTORS); ++i) mem->mdspeed[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_HINGES)*TP_SIZE_VEC4; ++i) mem->iniq[i] = TP_REAL(0.0);

#ifdef TP_DEBUG
	for(size_t i = 0; i < (TP_BODIES)*TP_SIZE_VEC6; ++i) mem->Fc[i] = TP_REAL(0.0);
	for(size_t i = 0; i < 3*(TP_FEET)*TP_SIZE_VEC6; ++i) mem->cinfo[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_FEET)*TP_SIZE_VEC6; ++i) mem->cplane[i] = TP_REAL(0.0);
	for(size_t i = 0; i < (TP_FEET); ++i) mem->cbody[i] = 0;
#endif
}

TP_FUNC_INLINE real_t * x(real_t *vec3)
{
	return vec3;
}

TP_FUNC_INLINE real_t _x(const real_t *vec3)
{
	return vec3[0];
}

TP_FUNC_INLINE real_t * y(real_t *vec3)
{
	return vec3+1;
}

TP_FUNC_INLINE real_t _y(const real_t *vec3)
{
	return vec3[1];
}

TP_FUNC_INLINE real_t * z(real_t *vec3)
{
	return vec3+2;
}

TP_FUNC_INLINE real_t _z(const real_t *vec3)
{
	return vec3[2];
}

TP_FUNC_INLINE real_t * q0(real_t *quatern)
{
	return quatern;
}

TP_FUNC_INLINE real_t _q0(const real_t *quatern)
{
	return quatern[0];
}

TP_FUNC_INLINE real_t * q1(real_t *quatern)
{
	return quatern+1;
}

TP_FUNC_INLINE real_t _q1(const real_t *quatern)
{
	return quatern[1];
}

TP_FUNC_INLINE real_t * q2(real_t *quatern)
{
	return quatern+2;
}

TP_FUNC_INLINE real_t _q2(const real_t *quatern)
{
	return quatern[2];
}

TP_FUNC_INLINE real_t * q3(real_t *quatern)
{
	return quatern+3;
}

TP_FUNC_INLINE real_t _q3(const real_t *quatern)
{
	return quatern[3];
}

TP_FUNC_INLINE real_t * ij(real_t *mtx33, index_t row, index_t col)
{
	return mtx33 + row*TP_SIZE_VEC3 + col;
}

TP_FUNC_INLINE real_t _ij(const real_t *mtx33, index_t row, index_t col)
{
	return *(mtx33 + row*TP_SIZE_VEC3 + col);
}

TP_FUNC_INLINE real_t * haxis(struct mem_t *m, index_t hinge_num)
{
	return m->haxes + hinge_num*2*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * haxis_num(struct mem_t *m, index_t hinge_num, index_t body_index)
{
	return m->haxes + hinge_num*2*TP_SIZE_VEC6 + 3*body_index;
}

TP_FUNC_INLINE real_t * ht0(struct mem_t *m, index_t hinge_num)
{
	return m->haxes + hinge_num*2*TP_SIZE_VEC6 + TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * ht1(struct mem_t *m, index_t hinge_num)
{
	return m->haxes + hinge_num*2*TP_SIZE_VEC6 + TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE real_t * hanchor(struct mem_t *m, index_t hinge_num, index_t body_index)
{
	return m->hanchors + hinge_num*TP_SIZE_VEC6 + 3*body_index;
}

TP_FUNC_INLINE real_t * pos(struct mem_t *m, index_t body)
{
	return m->q + body*(TP_SIZE_VEC3+TP_SIZE_VEC4);
}

TP_FUNC_INLINE real_t * vel(struct mem_t *m, index_t body)
{
	return m->v + body*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * omega(struct mem_t *m, index_t body)
{
	return m->v + body*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE real_t * quatern(struct mem_t *m, index_t body)
{
	return m->q + body*(TP_SIZE_VEC3+TP_SIZE_VEC4) + TP_SIZE_VEC3;
}

TP_FUNC_INLINE real_t * mi(struct mem_t *m, index_t body)
{
	return m->mi + body;
}
TP_FUNC_INLINE real_t _mi(struct mem_t *m, index_t body)
{
	return *(m->mi + body);
}

TP_FUNC_INLINE real_t * R(struct mem_t *m, index_t body)
{
	return m->R + body*3*TP_SIZE_VEC3;
}

TP_FUNC_INLINE real_t * Ibi(struct mem_t *m, index_t body)
{
	return m->Ibi + body*3*TP_SIZE_VEC3;
}

TP_FUNC_INLINE real_t * tFe(struct mem_t *m, index_t body)
{
	return m->Fe + body*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * aFe(struct mem_t *m, index_t body)
{
	return m->Fe + body*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE index_t * Jm(struct mem_t *m, index_t constraint, index_t body)
{
	return m->Jm + constraint*2 + body;
}

TP_FUNC_INLINE index_t _Jm(struct mem_t *m, index_t constraint, index_t body)
{
	return *(m->Jm + constraint*2 + body);
}

TP_FUNC_INLINE real_t * tJ(struct mem_t *m, index_t constraint, index_t body)
{
	return m->J + (constraint*2 + body)*(TP_SIZE_VEC6);
}

TP_FUNC_INLINE real_t * aJ(struct mem_t *m, index_t constraint, index_t body)
{
	return m->J + (constraint*2 + body)*(TP_SIZE_VEC6) + 3;
}

TP_FUNC_INLINE real_t * tB(struct mem_t *m, index_t constraint, index_t body)
{
	return m->B + (constraint*2 + body)*(TP_SIZE_VEC6);
}

TP_FUNC_INLINE real_t * aB(struct mem_t *m, index_t constraint, index_t body)
{
	return m->B + (constraint*2 + body)*(TP_SIZE_VEC6)+3;
}

TP_FUNC_INLINE real_t * ta(struct mem_t *m, index_t body)
{
	return m->a + body*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * aa(struct mem_t *m, index_t body)
{
	return m->a + body*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE real_t * lambda(struct mem_t *m, index_t constraint)
{
	return m->lambda + constraint;
}

TP_FUNC_INLINE real_t _lambda(struct mem_t *m, index_t constraint)
{
	return *(m->lambda + constraint);
}

TP_FUNC_INLINE real_t * lambda_min(struct mem_t *m, index_t constraint)
{
	return m->lambda_min + constraint;
}

TP_FUNC_INLINE real_t _lambda_min(struct mem_t *m, index_t constraint)
{
	return *(m->lambda_min + constraint);
}

TP_FUNC_INLINE real_t * lambda_max(struct mem_t *m, index_t constraint)
{
	return m->lambda_max + constraint;
}

TP_FUNC_INLINE real_t _lambda_max(struct mem_t *m, index_t constraint)
{
	return *(m->lambda_max + constraint);
}

TP_FUNC_INLINE real_t * d(struct mem_t *m, index_t constraint)
{
	return m->d + constraint;
}

TP_FUNC_INLINE real_t _d(struct mem_t *m, index_t constraint)
{
	return *(m->d + constraint);
}

TP_FUNC_INLINE real_t * rhs(struct mem_t *m, index_t constraint)
{
	return m->rhs + constraint;
}

TP_FUNC_INLINE real_t _rhs(struct mem_t *m, index_t constraint)
{
	return *(m->rhs + constraint);
}

TP_FUNC_INLINE real_t * mds(struct mem_t *m, index_t motor)
{
	return m->mdspeed + motor;
}

TP_FUNC_INLINE real_t _mds(struct mem_t *m, index_t motor)
{
	return *(m->mdspeed + motor);
}

TP_FUNC_INLINE index_t * mm(struct mem_t *m, index_t motor)
{
	return m->mm + motor;
}

TP_FUNC_INLINE index_t _mm(struct mem_t *m, index_t motor)
{
	return *(m->mm + motor);
}

TP_FUNC_INLINE real_t * iniquatern(struct mem_t *m, index_t hinge_num)
{
	return m->iniq + hinge_num*TP_SIZE_VEC4;
}

#ifdef TP_DEBUG
TP_FUNC_INLINE real_t * tFc(struct mem_t *m, index_t body)
{
	return m->Fc + body*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * aFc(struct mem_t *m, index_t body)
{
	return m->Fc + body*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE real_t * cpo(struct mem_t *m, index_t contact)
{
	return m->cinfo + contact*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * cno(struct mem_t *m, index_t contact)
{
	return m->cinfo + contact*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE real_t * cpl0(struct mem_t *m, index_t foot)
{
	return m->cplane + foot*TP_SIZE_VEC6;
}

TP_FUNC_INLINE real_t * cpl1(struct mem_t *m, index_t foot)
{
	return m->cplane + foot*TP_SIZE_VEC6 + 3;
}

TP_FUNC_INLINE index_t * cbdy(struct mem_t *m, index_t foot)
{
	return m->cbody + foot;
}

TP_FUNC_INLINE index_t _cbdy(struct mem_t *m, index_t foot)
{
	return *(m->cbody + foot);
}
#endif
