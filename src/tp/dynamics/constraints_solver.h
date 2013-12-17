
#pragma once

/**
 * @name Constraint Solver Functions
 *
 * The system is solved by the Projected Gauss-Seidel implementation found presented in:
 *
 * E. Catto. Iterative dynamics with temporal coherence. In Game Developer Conference, pages 1–24, 2005.
 *
 * As such, an iterative approach is used where the \f$i\f$th Lagrange multiplier change at each iteration as
 * \f[
 * 	\Delta \lambda_i = \frac{1}{d_i}(rhs - J_i\cdot a)
 * \f]
 * where
 * \f$J_i\f$ is the \f$i\f$th Jacobian row,
 * \f$a = B\lambda_0\f$, \f$B=M^{-1}J^{\mathrm{T}}\f$, \f$d\f$ is the diagonal of \f$JB\f$
 * and \f$rhs\f$ is the right hand side.
 *
 */
//@{

/** Computes the B vector.
 *
 * Computes \f$B = M^{-1}J^{T}\f$.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_B(struct mem_t *m)
{
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			index_t body = _Jm(m, s, bi);

			// Scale the translational components by 1/m = mi
			tp_vec3 _tJ;
			get_vec3(tJ(m, s, bi), _tJ);
			scale_to_vec3(_tJ, _mi(m, body));
			set_vec3(_tJ, tB(m, s, bi));

			// Set the rotational components to (Iwi = R*Ibi*Rt) * rotational components
			tp_mtx33 _Ibi;
			get_mtx33(Ibi(m, body), _Ibi);

			tp_mtx33 _R;
			get_mtx33(R(m, body), _R);

			tp_mtx33 IbiRT;
			mult_mtx33_mtx33T(IbiRT, _Ibi, _R);

			tp_mtx33 Ii;
			mult_mtx33_mtx33(Ii, _R, IbiRT);

			tp_vec3 _aJ;
			get_vec3(aJ(m, s, bi), _aJ);

			mult_to_mtx33_vec3(Ii, _aJ);
			set_vec3(_aJ, aB(m, s, bi));
		}
	}
}

/** Computes the \f$a\f$ vector.
 *
 * Computes \f$a = B\lambda\f$.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_a(struct mem_t *m)
{
	// a = B_{sp}\lambda
	// same alg. as computing Fc, but different storage, perhaps do
	// one function of this
	// Zero a
	for(int b = 0; b < (TP_BODIES); ++b)
	{
		*x(ta(m, b)) = TP_REAL(0.0);
		*y(ta(m, b)) = TP_REAL(0.0);
		*z(ta(m, b)) = TP_REAL(0.0);

		*x(aa(m, b)) = TP_REAL(0.0);
		*y(aa(m, b)) = TP_REAL(0.0);
		*z(aa(m, b)) = TP_REAL(0.0);
	}

	// First hinges (fixed) and motors (fixed)
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			index_t body = _Jm(m, s, bi);

			tp_vec3 _tB;
			get_vec3(tB(m, s, bi), _tB);
			scale_to_vec3(_tB, _lambda(m, s));

			*x(ta(m, body)) += _tB[0];
			*y(ta(m, body)) += _tB[1];
			*z(ta(m, body)) += _tB[2];

			tp_vec3 _aB;
			get_vec3(aB(m, s, bi), _aB);
			scale_to_vec3(_aB, _lambda(m, s));

			*x(aa(m, body)) += _aB[0];
			*y(aa(m, body)) += _aB[1];
			*z(aa(m, body)) += _aB[2];
		}
	}
}

/** Computes the constraint forces without separate storage.
 *
 * The constraint forces \f$F_c\f$ are computed as \f$F_c = J_T\lambda\f$. Using
 * this function saves storage, since it saves the forces, by adding them to the
 * external forces.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_Fc_add_to_Fe(struct mem_t *m)
{
	// F_c = J_T\lambda, add Fc to Fe

	// First hinges and motors (fixed number)
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			index_t body = _Jm(m, s, bi);

			tp_vec3 _tJ;
			get_vec3(tJ(m, s, bi), _tJ);
			scale_to_vec3(_tJ, _lambda(m, s));

			*x(tFe(m, body)) += _tJ[0];
			*y(tFe(m, body)) += _tJ[1];
			*z(tFe(m, body)) += _tJ[2];

			tp_vec3 _aJ;
			get_vec3(aJ(m, s, bi), _aJ);
			scale_to_vec3(_aJ, _lambda(m, s));

			*x(aFe(m, body)) += _aJ[0];
			*y(aFe(m, body)) += _aJ[1];
			*z(aFe(m, body)) += _aJ[2];
		}
	}
}

#ifdef TP_DEBUG
/** Computes the constraint forces.
 *
 * The constraint forces \f$F_c\f$ are computed as \f$F_c = J_T\lambda\f$. If
 * the TP_DEBUG macro is defined \f$F_c\f$, this function is available which makes
 * the constraint forces stored separately from the external forces.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_Fc(struct mem_t *m)
{
	// F_c = J_T\lambda, add Fc to Fe

	for(int i = 0; i < (TP_BODIES)*TP_SIZE_VEC6; ++i) m->Fc[i] = TP_REAL(0.0);

	// First hinges and motors (fixed number)
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			index_t body = _Jm(m, s, bi);

			tp_vec3 _tJ;
			get_vec3(tJ(m, s, bi), _tJ);
			scale_to_vec3(_tJ, _lambda(m, s));

			*x(tFc(m, body)) += _tJ[0];
			*y(tFc(m, body)) += _tJ[1];
			*z(tFc(m, body)) += _tJ[2];

			tp_vec3 _aJ;
			get_vec3(aJ(m, s, bi), _aJ);
			scale_to_vec3(_aJ, _lambda(m, s));

			*x(aFc(m, body)) += _aJ[0];
			*y(aFc(m, body)) += _aJ[1];
			*z(aFc(m, body)) += _aJ[2];
		}
	}
}
#endif

/** Computes the \f$d\f$ vector.
 *
 * Computes \f$d = JB\f$.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_d(struct mem_t *m)
{
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		real_t dii = TP_REAL(0.0);

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			tp_vec3 _tJ, _tB;
			get_vec3(tJ(m, s, bi), _tJ);
			get_vec3(tB(m, s, bi), _tB);
			dii += dot_vec3(_tJ, _tB);

			tp_vec3 _aJ, _aB;
			get_vec3(aJ(m, s, bi), _aJ);
			get_vec3(aB(m, s, bi), _aB);
			dii += dot_vec3(_aJ, _aB);
		}

		*d(m, s) = dii;
	}
}

/** Computes the \f$rhs\f$ vector.
 *
 * Computes \f$rhs = \frac{1}{\Delta t}\epsilon - \frac{1}{\Delta t}Ju - JM^{-1}F_e\f$.
 * Here \f$u\f$ represents the system velocity vector, and \f$F_e\f$ the external forces.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 * @param		dt			Simulation timestep.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void compute_rhs(struct mem_t *m, real_t dt)
{
	// rhs = \frac{1}{\Delta t}\epsilon - \frac{1}{\Delta t}JV - JM^{-1}F_e
	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		real_t JV = TP_REAL(0.0), JMiFe = TP_REAL(0.0);

		index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

		for(int bi = 1; bi >= stop_at_body; --bi)
		{
			index_t body = _Jm(m, s, bi);

			tp_vec3 _tJ;
			get_vec3(tJ(m, s, bi), _tJ);

			tp_vec3 _vel;
			get_vec3(vel(m, body), _vel);

			tp_vec3 _aJ;
			get_vec3(aJ(m, s, bi), _aJ);

			tp_vec3 _omega;
			get_vec3(omega(m, body), _omega);

			// JV (translational) + (rotational)
			JV += dot_vec3(_tJ, _vel) + dot_vec3(_aJ, _omega);

			tp_vec3 _tFe;
			get_vec3(tFe(m, body), _tFe);
			scale_to_vec3(_tFe, _mi(m, body));

			tp_vec3 _aFe;
			get_vec3(aFe(m, body), _aFe);

			tp_mtx33 _Ibi;
			get_mtx33(Ibi(m, body), _Ibi);

			tp_mtx33 _R;
			get_mtx33(R(m, body), _R);

			tp_mtx33 IbiRT;
			mult_mtx33_mtx33T(IbiRT, _Ibi, _R);

			tp_mtx33 Ii;
			mult_mtx33_mtx33(Ii, _R, IbiRT);

			mult_to_mtx33_vec3(Ii, _aFe);

			// JM^{-1}Fe (translational) + (rotational)
			JMiFe += dot_vec3(_tJ, _tFe) + dot_vec3(_aJ, _aFe);
		}

		*rhs(m, s) = -(TP_REAL(1.0)/dt) * JV - JMiFe;
	}

	// Error correction for hinges
	for(int h = 0; h < (TP_HINGES); ++h)
	{
		tp_vec3 anchors_world[2];

		for(int i = 0; i < 2; ++i)
		{
			index_t body = _Jm(m, 5*h, i);

			tp_mtx33 _R;
			get_mtx33(R(m, _Jm(m, 5*h, i)), _R);

			tp_vec3 _pos, _anchor_local;
			get_vec3(pos(m, body), _pos);
			get_vec3(hanchor(m, h, i), _anchor_local);

			mult_to_mtx33_vec3(_R, _anchor_local);
			add_vec3(anchors_world[i], _pos, _anchor_local, TP_REAL(1.0));
		}

		tp_vec3 error;
		add_vec3(error, anchors_world[1], anchors_world[0], TP_REAL(-1.0));

		*rhs(m, 5*h) 	+= (TP_ERP)/dt * error[0];
		*rhs(m, 5*h+1) 	+= (TP_ERP)/dt * error[1];
		*rhs(m, 5*h+2) 	+= (TP_ERP)/dt * error[2];

		// Axis rotation error
		tp_vec3 axis_local_1;
		get_vec3(haxis_num(m, h, 1), axis_local_1);

		tp_mtx33 _R1;
		get_mtx33(R(m, _Jm(m, 5*h, 1)), _R1);

		// Into world coords
		mult_to_mtx33_vec3(_R1, axis_local_1);

		tp_mtx33 _R0;
		get_mtx33(R(m, _Jm(m, 5*h, 0)), _R0);

		// Into body 0 coords
		mult_to_mtx33T_vec3(_R0, axis_local_1);

		tp_vec3 axis_local_0;
		get_vec3(haxis(m, h), axis_local_0);

		// u is in body 0 coords
		tp_vec3 u;
		cross_vec3(u, axis_local_0, axis_local_1);

		tp_vec3 t0, t1;
		get_vec3(ht0(m, h), t0);
		get_vec3(ht1(m, h), t1);

//		print_v3(u, "u", true);

		*rhs(m, 5*h+3) += (TP_ERP)/dt * dot_vec3(t0, u);
		*rhs(m, 5*h+4) += (TP_ERP)/dt * dot_vec3(t1, u);
	}

	// Add desired motor speed for the motor constraints
	for(int s = TP_HINGE_CONSTRAINTS, motor = 0; s < TP_HINGE_MOTOR_CONSTRAINTS; ++s, ++motor)
		*rhs(m, s) += _mds(m, motor)/dt;
}

/** Clamps a change to a float variable.
 *
 * Adds a change to a float, but keeps the float inside a given interval. Uses no if-statements.
 *
 * @param		old			Current value.
 * @param		delta		Change in value.
 * @param		min			Lower-end of interval.
 * @param		max			Upper-end of interval.
 * @return the clamped value.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
real_t clamp(real_t old, real_t delta, real_t min, real_t max)
{
	real_t new_val = old + delta;

	return (new_val > max)*(max - new_val) + new_val + (new_val < min)*(min - new_val);
}

/** Clamps a change to a float variable.
 *
 * Adds a change to a float, but keeps the float inside a given interval. Uses if-statements!
 *
 * @param		old			Current value.
 * @param		delta		Change in value.
 * @param		min			Lower-end of interval.
 * @param		max			Upper-end of interval.
 * @return the clamped value.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
real_t clamp2(real_t old, real_t delta, real_t min, real_t max)
{
	real_t new_val = old + delta;

	if(new_val > max)
		return max;
	else if(new_val < min)
		return min;

	return new_val;
}

/** Solves for Lagrange multiplier by Projected Gauss-Seidel.
 *
 * Computes \f$rhs = \frac{1}{\Delta t}\epsilon - \frac{1}{\Delta t}Ju - JM^{-1}F_e\f$.
 * Here \f$u\f$ represents the system velocity vector, and \f$F_e\f$ the external forces.
 *
 * @param		m				Pointer to the memory representing the simulation world.
 * @param		dt				Simulation timestep.
 * @param		num_iterations	Number of iterations.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void solve_for_lambda(struct mem_t *m, real_t dt, int num_iterations)
{
	// Projected Gauss-Seidel, as from Catto paper

	/* CUDA: Better to compute on empty rows, or better to quit
	 * and wait in a synchronization? Test? For now, empty rows!
	 */

	// Solves JB\lambda = rhs (J = sparse, B = sparse)
	compute_B(m);		// B = M^{-1}J^{T}
	compute_a(m);		// a = B\lambda_0
	compute_d(m);		// d = diag(JB)
	compute_rhs(m, dt);	// rhs = 1/dt*e - 1/dt*Jv - M^{-1}F_e

	for(int i = 0; i < num_iterations; ++i)
	{
//		real_t delta = 0.0;
		for(int s = 0; s < TP_CONSTRAINTS; ++s)
		{
			index_t stop_at_body = (s < TP_HINGE_MOTOR_CONSTRAINTS) ? 0 : 1;

			real_t tmp = TP_REAL(0.0);
			for(int bi = 1; bi >= stop_at_body; --bi)
			{
				index_t body = _Jm(m, s, bi);

				tp_vec3 _tJ;
				get_vec3(tJ(m, s, bi), _tJ);

				tp_vec3 _ta;
				get_vec3(ta(m, body), _ta);

				tmp += dot_vec3(_tJ, _ta);

				tp_vec3 _aJ;
				get_vec3(aJ(m, s, bi), _aJ);

				tp_vec3 _aa;
				get_vec3(aa(m, body), _aa);

				tmp += dot_vec3(_aJ, _aa);
			}

			// Fix to avoid d = 0
//			real_t dfix = _d(m, s) + (abs(_d(m, s) < 1e-7))*1e7;
//			real_t delta_lambda = (_rhs(m, s) - tmp) / dfix;

			real_t delta_lambda = TP_REAL(0.0);
			if(_d(m, s) > TP_REAL(1e-7) || _d(m, s) < TP_REAL(-1e-7))
				delta_lambda = (_rhs(m, s) - tmp) / _d(m, s);

//			if(s == 40)
//			delta = delta_lambda;
			// Limit lambda
			real_t new_lambda = clamp2(_lambda(m, s), delta_lambda, _lambda_min(m, s), _lambda_max(m, s));

			delta_lambda = new_lambda - _lambda(m, s);

			*lambda(m, s) += delta_lambda;

			for(int bi = 1; bi >= stop_at_body; --bi)
			{
				index_t body = _Jm(m, s, bi);

				*x(ta(m, body)) += delta_lambda * _x(tB(m, s, bi));
				*y(ta(m, body)) += delta_lambda * _y(tB(m, s, bi));
				*z(ta(m, body)) += delta_lambda * _z(tB(m, s, bi));

				*x(aa(m, body)) += delta_lambda * _x(aB(m, s, bi));
				*y(aa(m, body)) += delta_lambda * _y(aB(m, s, bi));
				*z(aa(m, body)) += delta_lambda * _z(aB(m, s, bi));
			}
		}
//		std::cout << delta/5.0 << std::endl;
	}
}

//@}
