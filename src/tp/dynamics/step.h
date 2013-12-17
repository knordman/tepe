
#pragma once


/** Steps a simulation world a dt amount of seconds.
 *
 * @param		m				Pointer to the memory representing world.
 * @param		dt				Size of timestep (seconds).
 * @param		num_iterations	Number of iterations to use in constraint force solver.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void step_world(struct mem_t *m, real_t dt, int num_iterations)
{
	// Update Jacobian for constraints (hinges)
	update_jacobian(m);

	// Compute contraint+contact lambdas
	solve_for_lambda(m, dt, num_iterations);

	// Add constraint+contact forces to external forces
	compute_Fc_add_to_Fe(m);

	#ifdef TP_DEBUG
	compute_Fc(m);
	#endif

	// Integrate with semi-implicit Euler
	for(int i = 0; i < (TP_BODIES); ++i)
	{
		// Velocity update, translational ---------------------------
		tp_vec3 _tFe;
		get_vec3(tFe(m, i), _tFe);

		*x(vel(m, i)) += dt * _mi(m, i) * _tFe[0];
		*y(vel(m, i)) += dt * _mi(m, i) * _tFe[1];
		*z(vel(m, i)) += dt * _mi(m, i) * _tFe[2];

		// Velocity update, rotational ------------------------------
		tp_mtx33 _Ibi;
		get_mtx33(Ibi(m, i), _Ibi);

		tp_mtx33 _R;
		get_mtx33(R(m, i), _R);

		tp_vec3 _aFe;
		get_vec3(aFe(m, i), _aFe);

		tp_mtx33 IbiRT;
		mult_mtx33_mtx33T(IbiRT, _Ibi, _R);

		tp_mtx33 Ii;
		mult_mtx33_mtx33(Ii, _R, IbiRT);

		mult_to_mtx33_vec3(Ii, _aFe);

		*x(omega(m, i)) += dt * _aFe[0];
		*y(omega(m, i)) += dt * _aFe[1];
		*z(omega(m, i)) += dt * _aFe[2];

// 		Gyroscopic update -------------------------------------------
//		tp_vec3 Iw, wxIw, _omega;
//		get_vec3(omega(m, i), _omega);
//		mult_mtx33_vec3(Iw, Iwi, _omega);
//		cross_vec3(wxIw, _omega, Iw);
//		std::cout << "|wxIw| = " << norm2_vec3(wxIw) << std::endl;
//		*x(omega(m, i)) += dt * (Iwite[0] - wxIw[0]);
//		*y(omega(m, i)) += dt * (Iwite[1] - wxIw[1]);
//		*z(omega(m, i)) += dt * (Iwite[2] - wxIw[2]);
//		-------------------------------------------------------------

		// Position update, translational ---------------------------
		*x(pos(m, i)) += dt * _x(vel(m, i));
		*y(pos(m, i)) += dt * _y(vel(m, i));
		*z(pos(m, i)) += dt * _z(vel(m, i));

		// Position update, rotational ------------------------------
		tp_vec3 _omega;
		get_vec3(omega(m, i), _omega);

		tp_quatern _quatern;
		get_quatern(quatern(m, i), _quatern);

		tp_quatern dq;
		mult_omega_quatern(dq, _omega, _quatern);

		_quatern[0] += dt * TP_REAL(0.5) * dq[0];
		_quatern[1] += dt * TP_REAL(0.5) * dq[1];
		_quatern[2] += dt * TP_REAL(0.5) * dq[2];
		_quatern[3] += dt * TP_REAL(0.5) * dq[3];
		normalize_quaternion(_quatern);

		set_quatern(_quatern, quatern(m, i));
		quaternion_to_rot_mtx33(_quatern, _R);
		set_mtx33(_R, R(m, i));

		// Zero external force and contact points for next step -----
		// Zero force+torque
		*x(tFe(m, i)) = TP_REAL(0.0);
		*y(tFe(m, i)) = TP_REAL(0.0);
		*z(tFe(m, i)) = TP_REAL(0.0);

		*x(aFe(m, i)) = TP_REAL(0.0);
		*y(aFe(m, i)) = TP_REAL(0.0);
		*z(aFe(m, i)) = TP_REAL(0.0);

		// Zero contact rows
		for(int s = TP_HINGE_MOTOR_CONSTRAINTS; s < TP_CONSTRAINTS; ++s)
		{
			tp_vec3 zero = {0.0, 0.0, 0.0};
			set_vec3(zero, tJ(m, s, 1));
			set_vec3(zero, aJ(m, s, 1));
		}
	}
}

