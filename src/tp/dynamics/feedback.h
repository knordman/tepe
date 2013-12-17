/*
 * feedback.h
 *
 *  Created on: Aug 1, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once


/** Returns the current angle of hinge joint.
 *
 * The angle of the hinge joint is the angle that describes how many radians
 * the two connected bodies are offset from the initial configuration. The
 * offset returned is within the interval -PI to PI.
 *
 * @param		m				Pointer to the memory representing the simulation world.
 * @param		hinge_num		Index of hinge.
 * @return current angle in radians.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
real_t hinge_angle(struct mem_t *m, index_t hinge_num)
{
	index_t b0 = _Jm(m, 5*hinge_num, 0);
	index_t b1 = _Jm(m, 5*hinge_num, 1);

	// Relative rotation between b0 and b1
	tp_quatern q0;
	get_quatern(quatern(m, b0), q0);

	tp_quatern q1;
	get_quatern(quatern(m, b1), q1);

	q0[1] *= TP_REAL(-1.0); q0[2] *= TP_REAL(-1.0); q0[3] *= TP_REAL(-1.0);

	tp_quatern dq;
	mult_quatern_quatern(dq, q0, q1);

	tp_quatern _iniquatern;
	get_quatern(iniquatern(m, hinge_num), _iniquatern);
	_iniquatern[1] *= TP_REAL(-1.0); _iniquatern[2] *= TP_REAL(-1.0); _iniquatern[3] *= TP_REAL(-1.0);

	tp_quatern hdq;
	mult_quatern_quatern(hdq, dq, _iniquatern);

	// Convert quaternion to angle (from ODE source)
	tp_vec3 axis;
	get_vec3(haxis(m, hinge_num), axis);
	tp_mtx33 R0;
	get_mtx33(R(m, b0), R0);
	mult_to_mtx33_vec3(R0, axis);

	real_t cost2 = hdq[0];
	real_t sint2 = sqrt(hdq[1]*hdq[1] + hdq[2]*hdq[2] + hdq[3]*hdq[3]);

	real_t theta = (dot_vec3(hdq + 1, axis) >= 0) ?
						(TP_REAL(2.0) * atan2(sint2, cost2) ) :
						(TP_REAL(2.0) * atan2(sint2, -cost2) );

	theta -= (theta > TP_PI)*(TP_REAL(2.0) * TP_PI);

	return theta;
}

/** Returns angle rate of hinge joint.
 *
 * @param		m				Pointer to the memory representing the simulation world.
 * @param		hinge_num		Index of hinge.
 * @return the angle rate of hinge joint in radians per second.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC
real_t hinge_angle_rate(struct mem_t *m, index_t hinge_num)
{
	index_t b0 = _Jm(m, 5*hinge_num, 0);
	index_t b1 = _Jm(m, 5*hinge_num, 1);

	tp_mtx33 R0;
	get_mtx33(R(m, b0), R0);

	// The axis is stored in the ref. frame of body 0
	tp_vec3 axis;
	get_vec3(haxis(m, hinge_num), axis);

	// To world frame
	mult_to_mtx33_vec3(R0, axis);

	tp_vec3 omega0;
	get_vec3(omega(m, b0), omega0);

	real_t rate = -dot_vec3(axis, omega0);

	tp_vec3 omega1;
	get_vec3(omega(m, b1), omega1);

	rate += dot_vec3(axis, omega1);

	return rate;
}


TP_FUNC_INLINE
real_t motor_torque(struct mem_t *m, index_t motor_num)
{
	return _lambda(m, TP_HINGE_CONSTRAINTS + motor_num);
}
