
#pragma once


/** Configures a motor for a hinge joint.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 * @param		motor		Index of motor to configure.
 * @param		hinge		Index of hinge the motor is connected to.
 * @param		max_torque	Maximum torque in Nm for motor.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void add_motor(
		struct mem_t *m,
		index_t motor,
		index_t hinge,
		real_t max_torque)
{
	index_t constraint_num = TP_HINGE_CONSTRAINTS + motor;

	*mm(m, motor) = hinge;
	*lambda_max(m, constraint_num) = max_torque;
	*lambda_min(m, constraint_num) = -max_torque;

	index_t hinge_constraint_num = 5*hinge;
	index_t body0 = _Jm(m, hinge_constraint_num, 0);
	index_t body1 = _Jm(m, hinge_constraint_num, 1);

	*Jm(m, constraint_num, 0) = body0;
	*Jm(m, constraint_num, 1) = body1;

	tp_quatern q0;
	get_quatern(quatern(m, body0), q0);

	tp_quatern q1;
	get_quatern(quatern(m, body1), q1);

	// Compute difference in rotation
	q0[1] *= TP_REAL(-1.0); q0[2] *= TP_REAL(-1.0); q0[3] *= TP_REAL(-1.0);

	tp_quatern dq;
	mult_quatern_quatern(dq, q0, q1);

	set_quatern(dq, iniquatern(m, hinge));
}

/** Configures a hinge joint between two bodies.
 *
 * @param		m					Pointer to the memory representing the simulation world.
 * @param		hinge_num			Index of hinge to configure.
 * @param		b0					Index of first body of hinge joint.
 * @param		b1					Index of second body of hinge joint.
 * @param		anchor_world_coord	Hinge anchor in world coordinates.
 * @param		axis_world_coord	Direction of hinge axis in world coordinates.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void create_hinge(
		struct mem_t *m,
		index_t hinge_num,				// pos in J
		index_t b0, 					// needed for the pos and vel vectors, needs to be stored for updating
		index_t b1,						// -''-
		tp_vec3 anchor_world_coord, 	// needs to be stored for updating
		tp_vec3 axis_world_coord)		// needs to be stored for updating
{
	// Mark the bodies in the Jacobian map
	for(int s = 5*hinge_num; s < 5*hinge_num + 5; ++s)
	{
		*Jm(m, s, 0) = b0;
		*Jm(m, s, 1) = b1;
	}

	for(int b = 0; b < 2; ++b)
	{
		// Set hinge anchors ----------------------------------------
		index_t body = (b == 0) ? b0 : b1;

		tp_mtx33 _R;
		get_mtx33(R(m, body), _R);

		tp_vec3 _pos;
		get_vec3(pos(m, body), _pos);

		tp_vec3 aw;
		aw[0] = anchor_world_coord[0];
		aw[1] = anchor_world_coord[1];
		aw[2] = anchor_world_coord[2];
		add_to_vec3(aw, _pos, TP_REAL(-1.0));

		mult_to_mtx33T_vec3(_R, aw);
		set_vec3(aw, hanchor(m, hinge_num, b));

		// Set rotation axis ----------------------------------------
		tp_vec3 aaw;
		aaw[0] = axis_world_coord[0];
		aaw[1] = axis_world_coord[1];
		aaw[2] = axis_world_coord[2];
		normalize_vec3(aaw);

		tp_vec3 axis_local;
		mult_mtx33T_vec3(axis_local, _R, aaw);

		set_vec3(axis_local, haxis_num(m, hinge_num, b));
	}

	// Construct tangent base to axis -------------------------------
	tp_vec3 axis;
	get_vec3(haxis(m, hinge_num), axis);

	tp_vec3 t0;
	for(int i = 0; i < 3; ++i)
	{
		if(axis[i]*axis[i] < TP_REAL(0.01)) continue;

		int j = (i == 0) ? 1 : (i == 2) ? 0 : 2;

		t0[i] = axis[j];
		t0[j] = -axis[i];
		t0[3-i-j] = TP_REAL(0.0);
		normalize_vec3(t0);
	}
	set_vec3(t0, ht0(m, hinge_num));

	tp_vec3 t1;
	cross_vec3(t1, t0, axis);
	set_vec3(t1, ht1(m, hinge_num));

	// Add constant translational parts to Jacobian -----------------
	*x(tJ(m, 5*hinge_num, 0)) 		= TP_REAL(1.0);
	*y(tJ(m, 5*hinge_num+1, 0))		= TP_REAL(1.0);
	*z(tJ(m, 5*hinge_num+2, 0)) 	= TP_REAL(1.0);

	*x(tJ(m, 5*hinge_num, 1)) 		= TP_REAL(-1.0);
	*y(tJ(m, 5*hinge_num+1, 1)) 	= TP_REAL(-1.0);
	*z(tJ(m, 5*hinge_num+2, 1)) 	= TP_REAL(-1.0);
}

/** Updates the Jacobian after a timestep.
 *
 * Every time when the simulation has been progressed one timestep, positions
 * and velocities of the bodies has (most likely) changed. Because of this the
 * Jacobian entries for the constraining joints are old and need to be updated.
 * This functions does exactly that.
 *
 * @param		m			Pointer to the memory representing the simulation world.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
void update_jacobian(struct mem_t *m)
{
	for(int h = 0; h < (TP_HINGES); ++h)
	{
		// Add rotational parts to Jacobian, (R * local_anchor)^x ---
		tp_vec3 anchors_world[2];

		for(int b = 0; b < 2; ++b)
		{
			index_t body = _Jm(m, 5*h, b);

			tp_mtx33 _R;
			get_mtx33(R(m, body), _R);

			tp_vec3 anchor_local;
			get_vec3(hanchor(m, h, b), anchor_local);

			mult_mtx33_vec3(anchors_world[b], _R, anchor_local);
		}

		// Body 0
		/*          x    y      z
		 * -a x   =	0 	 a2    -a1     5*h
		 * 		   -a2	 0	    a0	   5*h + 1
		 * 		    a1  -a0     0      5*h + 2
		 */

		*x(aJ(m, 5*h, 0)) 		= TP_REAL(0.0);
		*y(aJ(m, 5*h, 0)) 		= anchors_world[0][2];
		*z(aJ(m, 5*h, 0)) 		= -anchors_world[0][1];

		*x(aJ(m, 5*h+1, 0)) 	= -anchors_world[0][2];
		*y(aJ(m, 5*h+1, 0)) 	= TP_REAL(0.0);
		*z(aJ(m, 5*h+1, 0)) 	= anchors_world[0][0];

		*x(aJ(m, 5*h+2, 0)) 	= anchors_world[0][1];
		*y(aJ(m, 5*h+2, 0)) 	= -anchors_world[0][0];
		*z(aJ(m, 5*h+2, 0)) 	= TP_REAL(0.0);

		// Body 1
		/*          x    y      z
		 * a x   =	0 	-a2 	a1     5*h
		 * 			a2	 0	   -a0	   5*h + 1
		 * 		   -a1   a0     0      5*h + 2
		 */

		*x(aJ(m, 5*h, 1)) 		= TP_REAL(0.0);
		*y(aJ(m, 5*h, 1)) 		= -anchors_world[1][2];
		*z(aJ(m, 5*h, 1)) 		= anchors_world[1][1];

		*x(aJ(m, 5*h+1, 1)) 	= anchors_world[1][2];
		*y(aJ(m, 5*h+1, 1)) 	= TP_REAL(0.0);
		*z(aJ(m, 5*h+1, 1)) 	= -anchors_world[1][0];

		*x(aJ(m, 5*h+2, 1)) 	= -anchors_world[1][1];
		*y(aJ(m, 5*h+2, 1)) 	= anchors_world[1][0];
		*z(aJ(m, 5*h+2, 1)) 	= TP_REAL(0.0);


		// Jacobian rows for axis tangent base ----------------------
		// Axis of body 0 is used
		tp_mtx33 _R0;
		get_mtx33(R(m, _Jm(m, 5*h, 0)), _R0);

		tp_vec3 t[2];

		get_vec3(ht0(m, h), t[0]);
		mult_to_mtx33_vec3(_R0, t[0]);

		get_vec3(ht1(m, h), t[1]);
		mult_to_mtx33_vec3(_R0, t[1]);

		for(int i = 0; i < 2; ++i)
		{
			*x(aJ(m, 5*h+3+i, 0)) 		=  t[i][0];
			*y(aJ(m, 5*h+3+i, 0)) 		=  t[i][1];
			*z(aJ(m, 5*h+3+i, 0)) 		=  t[i][2];

			*x(aJ(m, 5*h+3+i, 1)) 		= -t[i][0];
			*y(aJ(m, 5*h+3+i, 1)) 		= -t[i][1];
			*z(aJ(m, 5*h+3+i, 1)) 		= -t[i][2];
		}
	}

	for(int s = TP_HINGE_CONSTRAINTS, motor = 0; s < TP_HINGE_MOTOR_CONSTRAINTS; ++s, ++motor)
	{
		index_t hinge = _mm(m, motor);
		index_t hinge_body = _Jm(m, s, 0);

		tp_vec3 axis;
		get_vec3(haxis(m, hinge), axis);

		tp_mtx33 _R;
		get_mtx33(R(m, hinge_body), _R);

		mult_to_mtx33_vec3(_R, axis);

		*x(aJ(m, s, 0)) = -axis[0];
		*y(aJ(m, s, 0)) = -axis[1];
		*z(aJ(m, s, 0)) = -axis[2];

		*x(aJ(m, s, 1)) = axis[0];
		*y(aJ(m, s, 1)) = axis[1];
		*z(aJ(m, s, 1)) = axis[2];
	}
}
