
#pragma once


/** Configures the mass and inertia of a box with uniform density.
 *
 * @param		mass			Mass of the box.
 * @param		mi				Pointer to memory location where inverse mass is to be stored.
 * @param		xlen			The x-dimension of the box.
 * @param		ylen			The y-dimension of the box.
 * @param		zlen			The z-dimension of the box.
 * @param		Ibi				Pointer to memory location where inertia tensor is to be stored.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC_INLINE
void set_box_inertia(
		real_t mass,
		real_t *mi,
		real_t xlen,
		real_t ylen,
		real_t zlen,
		real_t *Ibi)
{
	*mi = TP_REAL(1.0)/mass;

	*ij(Ibi, 0, 0) = TP_REAL(12.0)/(mass * (ylen*ylen + zlen*zlen));
	*ij(Ibi, 0, 1) = TP_REAL(0.0);
	*ij(Ibi, 0, 2) = TP_REAL(0.0);

	*ij(Ibi, 1, 0) = TP_REAL(0.0);
	*ij(Ibi, 1, 1) = TP_REAL(12.0)/(mass * (xlen*xlen + zlen*zlen));
	*ij(Ibi, 1, 2) = TP_REAL(0.0);

	*ij(Ibi, 2, 0) = TP_REAL(0.0);
	*ij(Ibi, 2, 1) = TP_REAL(0.0);
	*ij(Ibi, 2, 2) = TP_REAL(12.0)/(mass * (xlen*xlen + ylen*ylen));
}

/** Configures the mass and inertia of a box with uniform density.
 *
 * @param		mass			Mass of the box.
 * @param		mi				Pointer to memory location where inverse mass is to be stored.
 * @param		dim				3D vector containing box dimensions.
 * @param		Ibi				Pointer to memory location where inertia tensor is to be stored.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC_INLINE
void set_box_inertia(
		real_t mass,
		real_t *mi,
		const tp_vec3 dim,
		real_t *Ibi)
{
	set_box_inertia(
			mass,
			mi,
			dim[0],
			dim[1],
			dim[2],
			Ibi);
}

/** Configures the mass and inertia of a cylinder with uniform density.
 *
 * @param		mass			Mass of the box.
 * @param		mi				Pointer to memory location where inverse mass is to be stored.
 * @param		radius			Cylinder radius in meter.
 * @param		height			Cylinder height in meter.
 * @param		Ibi				Pointer to memory location where inertia tensor is to be stored.
 *
 * @ingroup tp-dynamics
 */
TP_FUNC_INLINE
void set_cylinder_inertia(
		real_t mass,
		real_t *mi,
		real_t radius,
		real_t height,
		real_t *Ibi)
{
	*mi = TP_REAL(1.0)/mass;

	*ij(Ibi, 0, 0) = TP_REAL(12.0)/(mass * (3*radius*radius + height*height));
	*ij(Ibi, 0, 1) = TP_REAL(0.0);
	*ij(Ibi, 0, 2) = TP_REAL(0.0);

	*ij(Ibi, 1, 0) = TP_REAL(0.0);
	*ij(Ibi, 1, 1) = TP_REAL(12.0)/(mass * (3*radius*radius + height*height));
	*ij(Ibi, 1, 2) = TP_REAL(0.0);

	*ij(Ibi, 2, 0) = TP_REAL(0.0);
	*ij(Ibi, 2, 1) = TP_REAL(0.0);
	*ij(Ibi, 2, 2) = TP_REAL(2.0)/(mass * radius*radius);
}
