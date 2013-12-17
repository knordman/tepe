/*
 * collision.h
 *
 *  Created on: Jul 31, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once


TP_FUNC
real_t get_terrain_height(const tp_vec3 pos)
{
	return 0.0;
}

/**
 * Collides a body as a cylindrical foot against the terrain. The body position
 * is considered to be geometrical center of the cylinder. If a collision is
 * detected necessary constraints are added to the Jacobian.
 *
 * @param		m				Pointer to the memory representing the simulation world.
 * @param		cyl_radius		Radius of the cylinder.
 * @param		cyl_height		Height of the cylinder.
 * @param		contacts_offset	Offset for the contact constraints rows in the Jacobian matrix.
 * @param		foot_body		Index for body to collide as foot, in the interval [0, #TP_BODIES-1].
 * @return Number of constraint rows added.
 *
 * @ingroup tp-collision
 */
TP_FUNC
int collide_foot_cylinder_tri(
		struct mem_t *m,
		real_t cyl_radius,
		real_t cyl_height,
		index_t contacts_offset,
		index_t foot_body)
{
	const real_t SIN30 = TP_REAL(0.5);
	const real_t COS30 = TP_REAL(0.8660254037);

	tp_vec3 _pos;
	get_vec3(pos(m, foot_body), _pos);

	real_t check_point	= _pos[2] - cyl_height * TP_REAL(0.5);
	real_t terrain_height = get_terrain_height(_pos);

	if(check_point > terrain_height) return 0;

	tp_vec3 contact_point_lc[3];
	contact_point_lc[0][0] = -cyl_radius;
	contact_point_lc[0][1] = TP_REAL(0.0);
	contact_point_lc[0][2] = -cyl_height*TP_REAL(0.5);

	contact_point_lc[1][0] = SIN30*cyl_radius;
	contact_point_lc[1][1] = COS30*cyl_radius;
	contact_point_lc[1][2] = -cyl_height*TP_REAL(0.5);

	contact_point_lc[2][0] = SIN30*cyl_radius;
	contact_point_lc[2][1] = -COS30*cyl_radius;
	contact_point_lc[2][2] = -cyl_height*TP_REAL(0.5);

	tp_mtx33 _R;
	get_mtx33(R(m, foot_body), _R);

	real_t h[3];
	tp_vec3 contact_point_wc[3];
	tp_vec3 contact_point_wf[3];

	// Get world vectors of contact points and heights at the points
	for(int cpoint = 0; cpoint < 3; ++cpoint)
	{
		mult_mtx33_vec3(contact_point_wf[cpoint], _R, contact_point_lc[cpoint]);
		add_vec3(contact_point_wc[cpoint], contact_point_wf[cpoint], _pos, TP_REAL(1.0));
		h[cpoint] = get_terrain_height(contact_point_wc[cpoint]);
	}

	tp_vec3 t1;
	t1[0] = contact_point_wc[1][0] - contact_point_wc[0][0];
	t1[1] = contact_point_wc[1][1] - contact_point_wc[0][1];
	t1[2] = h[1] - h[0];

	tp_vec3 t2;
	t2[0] = contact_point_wc[2][0] - contact_point_wc[0][0];
	t2[1] = contact_point_wc[2][1] - contact_point_wc[0][1];
	t2[2] = h[2] - h[0];

	tp_vec3 normal;
	cross_vec3(normal, t2, t1);
	normalize_vec3(normal);

	// Add contact points to Jacobian -------------------------------
	const index_t s = TP_HINGE_MOTOR_CONSTRAINTS + contacts_offset;

	// Add normal components
	for(int cpoint = 0; cpoint < 3; ++cpoint)
	{
		*Jm(m, s+cpoint, 0) = -1;
		*Jm(m, s+cpoint, 1) = foot_body;

		*x(tJ(m, s+cpoint, 1)) = normal[0];
		*y(tJ(m, s+cpoint, 1)) = normal[1];
		*z(tJ(m, s+cpoint, 1)) = normal[2];

		tp_vec3 cxn;
		cross_vec3(cxn, contact_point_wf[cpoint], normal);

		*x(aJ(m, s+cpoint, 1)) = cxn[0];
		*y(aJ(m, s+cpoint, 1)) = cxn[1];
		*z(aJ(m, s+cpoint, 1)) = cxn[2];

		*lambda_min(m, s+cpoint) = TP_REAL(0.0);
	}


	// Make contact non-slippery
	const real_t SIN45 = TP_REAL(0.7071067811);
	const real_t COS45 = TP_REAL(0.7071067811);

	tp_vec3 contact_tangent[2];
	contact_tangent[0][0] = COS45;
	contact_tangent[0][1] = SIN45;
	contact_tangent[0][2] = TP_REAL(0.0);
	mult_to_mtx33_vec3(_R, contact_tangent[0]);

	contact_tangent[1][0] = COS45;
	contact_tangent[1][1] = -SIN45;
	contact_tangent[1][2] = TP_REAL(0.0);
	mult_to_mtx33_vec3(_R, contact_tangent[1]);

//	for(int t = 0; t < 2; ++t)
//	{
//		*Jm(m, s+3+t, 0) 	= -1;
//		*Jm(m, s+3+t, 1) 	= foot_body;
//
//		*x(tJ(m, s+3+t, 1)) = contact_tangent[t][0];
//		*y(tJ(m, s+3+t, 1)) = contact_tangent[t][1];
//		*z(tJ(m, s+3+t, 1)) = contact_tangent[t][2];
//
//		/* The next constraint makes the mass center of the foot not move
//		 * in the tangent directions if there is contact, it should
//		 * really be a touching point, but since feet are flat, this
//		 * saves a cross product
//		 */
//		*x(aJ(m, s+3+t, 1)) = 0.0;
//		*y(aJ(m, s+3+t, 1)) = 0.0;
//		*z(aJ(m, s+3+t, 1)) = 0.0;
//	}


#ifdef TP_DEBUG
	index_t num_contact = contacts_offset / TP_CONTACT_CONSTRAINTS;
	index_t cinfo_offset = num_contact*TP_CONTACTS_ON_FOOT;

	for(int c = 0; c < 3; ++c)
	{
		set_vec3(contact_point_wc[c], cpo(m, cinfo_offset+c));
		set_vec3(normal, cno(m, cinfo_offset+c));
	}

	*cbdy(m, num_contact) = foot_body;
	set_vec3(contact_tangent[0], cpl0(m, num_contact));
	set_vec3(contact_tangent[1], cpl1(m, num_contact));
#endif

	return 5;
}

/*
TP_FUNC
int collide_foot_cylinder_quad(
		struct mem_t *m,
		real_t cyl_radius,
		real_t cyl_height,
		index_t contacts_offset,
		index_t foot_body)
{
	tp_vec3 _pos;
	get_vec3(pos(m, foot_body), _pos);

	real_t check_point	= _pos[2] - cyl_height * 0.5;
	real_t terrain_height = get_terrain_height(_pos);

	if(check_point > terrain_height) return 0;

	tp_vec3 c[4];
	c[0][0] = -cyl_radius;
	c[0][1] = 0.0;
	c[0][2] = -cyl_height*0.5;

	c[1][0] = cyl_radius;
	c[1][1] = 0.0;
	c[1][2] = -cyl_height*0.5;

	c[2][0] = 0.0;
	c[2][1] = -cyl_radius;
	c[2][2] = -cyl_height*0.5;

	c[3][0] = 0.0;
	c[3][1] = cyl_radius;
	c[3][2] = -cyl_height*0.5;

	tp_mtx33 _R;
	get_mtx33(R(m, foot_body), _R);

	real_t h[4];
	tp_vec3 cw[4];
	// Get world vectors of contact points
	for(int cpo = 0; cpo < 4; ++cpo)
	{
		mult_to_mtx33_vec3(_R, c[cpo]);
		add_vec3(cw[cpo], c[cpo], _pos, 1.0);

		h[cpo] = get_terrain_height(cw[cpo]);
	}

	tp_vec3 t1;
	t1[0] = cw[3][0] - cw[0][0];
	t1[1] = cw[3][1] - cw[0][1];
	t1[2] = h[1] - h[0];

	tp_vec3 t2;
	t2[0] = cw[2][0] - cw[0][0];
	t2[1] = cw[2][1] - cw[0][1];
	t2[2] = h[2] - h[0];

	tp_vec3 normal;
	cross_vec3(normal, t2, t1);
	normalize_vec3(normal);

//	draw_contact(cw[0], normal);
//	draw_contact(cw[1], normal);
//	draw_contact(cw[2], normal);
//	draw_contact(cw[3], normal);


	// Add contact points to Jacobian -------------------------------
	index_t s = TP_HINGE_MOTOR_CONSTRAINTS + contacts_offset;

	for(int cpo = 0; cpo < 4; ++cpo)
	{
		*Jm(m, s+cpo, 0) = -1;
		*Jm(m, s+cpo, 1) = foot_body;

		*x(tJ(m, s+cpo, 1)) = normal[0];
		*y(tJ(m, s+cpo, 1)) = normal[1];
		*z(tJ(m, s+cpo, 1)) = normal[2];

		tp_vec3 cxn;
		cross_vec3(cxn, c[cpo], normal);

		*x(aJ(m, s+cpo, 1)) = cxn[0];
		*y(aJ(m, s+cpo, 1)) = cxn[1];
		*z(aJ(m, s+cpo, 1)) = cxn[2];

		*lambda_min(m, s+cpo) = 0.0;
	}

	return 4;
}
*/
