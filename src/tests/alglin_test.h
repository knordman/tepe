/*
 * eqnsolve_test.h
 *
 *  Created on: Jul 12, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cxxtest/TestSuite.h>

#define TP_REAL(X) ((real_t)(X))
#include <tp/types/default.h>
#include <tp/alglin.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace Eigen;

class alglin_test : public CxxTest::TestSuite
{
public:
	void copy_vec(Vector3d &v, tp_vec3 tv)
	{
		tv[0] = v(0);
		tv[1] = v(1);
		tv[2] = v(2);
	}


	void copy_mtx(Matrix3d &m, tp_mtx33 tm)
	{
		for(int i = 0; i < 3; ++i)
			for(int j = 0; j < 3; ++j)
				tm[i*TP_SIZE_VEC3+j] = m(i,j);
	}


	void copy_quaternion(Quaterniond &q, tp_quatern tq)
	{
		tq[0] = q.w();
		tq[1] = q.x();
		tq[2] = q.y();
		tq[3] = q.z();
	}


	void check_equal(Quaterniond q, tp_quatern tq)
	{
		TS_ASSERT_DELTA(q.w(), tq[0], 1e-7);
		TS_ASSERT_DELTA(q.x(), tq[1], 1e-7);
		TS_ASSERT_DELTA(q.y(), tq[2], 1e-7);
		TS_ASSERT_DELTA(q.z(), tq[3], 1e-7);
	}


	void check_differs(Quaterniond q, tp_quatern tq)
	{
		TS_ASSERT_DIFFERS(q.w(), tq[0]);
		TS_ASSERT_DIFFERS(q.x(), tq[1]);
		TS_ASSERT_DIFFERS(q.y(), tq[2]);
		TS_ASSERT_DIFFERS(q.z(), tq[3]);
	}


	void check_equal(Vector3d v, tp_vec3 tv)
	{
		for(int i = 0; i < 3; ++i)
			TS_ASSERT_DELTA(tv[i], v(i), 1e-7);
	}


	void check_equal(Matrix3d m, tp_mtx33 tm)
	{
		for(int i = 0; i < 3; ++i)
			for(int j = 0; j < 3; ++j)
				TS_ASSERT_DELTA(tm[i*TP_SIZE_VEC3+j], m(i,j), 1e-7);
	}


	void stage_mtx(Matrix3d &m, Vector3d &v,
			tp_mtx33 tm, tp_vec3 tv)
	{
		m = Matrix3d::Random(3,3);
		v = Vector3d::Random(3,1);

		copy_mtx(m, tm);
		copy_vec(v, tv);
	}


	void stage(Vector3d &v1, Vector3d &v2, Vector3d &v3,
			tp_vec3 tv1, tp_vec3 tv2, tp_vec3 tv3)
	{
		v1 = Vector3d::Random();
		v2 = Vector3d::Random();
		v3 = Vector3d::Random();
		copy_vec(v1, tv1);
		copy_vec(v2, tv2);
		copy_vec(v3, tv3);
	}

	/** Tests addition of 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_add()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		v3 = v1 + v2;
		add_vec3(tv3, tv1, tv2, 1.0);
		check_equal(v3, tv3);

		v3 = v1 - v2;
		add_to_vec3(tv1, tv2, -1.0);
		check_equal(v3, tv1);
	}

	/** Tests the dot product 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_dot()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		TS_ASSERT_DELTA(v1.dot(v2), dot_vec3(tv1, tv2), 1e-7);
	}

	/** Tests the scaling of 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_scale()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		v3 = v1 * 3.14156;
		scale_vec3(tv3, tv1, 3.14156);
		check_equal(v3, tv3);

		scale_to_vec3(tv1, 3.14156);
		check_equal(v3, tv1);
	}

	/** Tests computing L2 norm of 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_norm2()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		TS_ASSERT_EQUALS(v1.norm(), norm2_vec3(tv1));
		TS_ASSERT_EQUALS(v2.norm(), norm2_vec3(tv2));
		TS_ASSERT_EQUALS(v3.norm(), norm2_vec3(tv3));
	}

	/** Tests normalization of 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_normalize()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		v1.normalize();
		v2.normalize();
		v3.normalize();

		normalize_vec3(tv1);
		normalize_vec3(tv2);
		normalize_vec3(tv3);

		check_equal(v1, tv1);
		check_equal(v2, tv2);
		check_equal(v3, tv3);
	}

	/** Tests computing the cross product of 3D vectors.
	 *
	 * @ingroup tp-tests
	 */
	void test_cross()
	{
		Vector3d v1, v2, v3;
		tp_vec3 tv1, tv2, tv3;
		stage(v1, v2, v3, tv1, tv2, tv3);

		v3 = v1.cross(v2);
		cross_vec3(tv3, tv1, tv2);
		check_equal(v3, tv3);

		cross_to_vec3(tv1, tv2);
		check_equal(v3, tv1);
	}

	/** Tests setting a 3x3 matrix to identity.
	 *
	 * @ingroup tp-tests
	 */
	void test_mtx_set_to_ident()
	{
		Matrix3d m = Matrix3d::Identity();
		tp_mtx33 tm;

		set_to_ident_mtx33(tm);

		check_equal(m, tm);
	}

	/** Tests multiplication of a 3x3 matrix with a 3x1 vector.
	 *
	 * @ingroup tp-tests
	 */
	void test_mtx_mult()
	{
		Matrix3d m;
		Vector3d v;
		tp_mtx33 tm;
		tp_vec3 tv;
		stage_mtx(m, v, tm, tv);

		Vector3d v2;
		tp_vec3 tv2;

		v2 = m * v;
		mult_mtx33_vec3(tv2, tm, tv);
		check_equal(v2, tv2);

		mult_to_mtx33_vec3(tm, tv);
		check_equal(v2, tv);
	}

	/** Tests multiplying two 3x3 matrices.
	 *
	 * @ingroup tp-tests
	 */
	void test_mtx2_mult()
	{
		Matrix3d m1 = Matrix3d::Random();
		Matrix3d m2 = Matrix3d::Random();
		Matrix3d m3;
		tp_mtx33 tm1, tm2, tm3;
		copy_mtx(m1, tm1);
		copy_mtx(m2, tm2);

		m3 = m1 * m2;
		mult_mtx33_mtx33(tm3, tm1, tm2);
		check_equal(m3, tm3);

		m3 = m2 * m1;
		mult_mtx33_mtx33(tm3, tm2, tm1);
		check_equal(m3, tm3);
	}

	/** Tests multiplying a 3x3 matrix with the transpose of another 3x3 matrix.
	 *
	 * @ingroup tp-tests
	 */
	void test_mtx2_multT()
	{
		Matrix3d m1 = Matrix3d::Random();

		tp_mtx33 tm1;
		copy_mtx(m1, tm1);

		Matrix3d m3 = m1 * m1.transpose();

		tp_mtx33 tm3;
		mult_mtx33_mtx33T(tm3, tm1, tm1);
		check_equal(m3, tm3);
	}

	/** Tests multiplying the transpose of a 3x3 matrix with a 3x1 vector.
	 *
	 * @ingroup tp-tests
	 */
	void test_mult_mtxT_vec3()
	{
		Matrix3d m;
		Vector3d v;
		tp_mtx33 tm;
		tp_vec3 tv;
		stage_mtx(m, v, tm, tv);

		Vector3d v2 = m.transpose() * v;

		tp_vec3 tv2;
		mult_mtx33T_vec3(tv2, tm, tv);

		check_equal(v2, tv2);

		mult_to_mtx33T_vec3(tm, tv);

		check_equal(v2, tv);
	}

	/** Tests transposing 3x3 matrix.
	 *
	 * @ingroup tp-tests
	 */
	void test_transpose()
	{
		Matrix3d m1 = Matrix3d::Random();
		Matrix3d m2;
		tp_mtx33 tm1, tm2;
		copy_mtx(m1, tm1);

		m2 = m1.transpose();
		transpose_mtx33(tm2, tm1);
		check_equal(m2, tm2);

		transpose_to_mtx33(tm1);
		check_equal(m2, tm1);
	}

	/** Tests computing the special quaternion, angular velocity product.
	 *
	 * @ingroup tp-tests
	 */
	void test_q_dot_w()
	{
		Quaterniond q = Quaterniond(1, 2, 3, 4);
		Quaterniond w = Quaterniond(0, 7, 1, 89);

		tp_quatern tq;
		copy_quaternion(q, tq);
		tp_vec3 tw = {7, 1, 89};

		tp_quatern tr;
		mult_omega_quatern(tr, tw, tq);

		Quaterniond r = w*q;

		check_equal(r, tr);
	}

	/** Tests transforming a quaternion to a rotation matrix.
	 *
	 * @ingroup tp-tests
	 */
	void test_q_to_r()
	{
		Quaterniond q = Quaterniond(0.707107, 0, -0.707107, 0);
		q.normalize();
		tp_quatern tq;
		copy_quaternion(q, tq);

		Matrix3d R = q.toRotationMatrix();

		tp_mtx33 tm;
		quaternion_to_rot_mtx33(tq, tm);

		check_equal(R, tm);
	}

	/** Tests the normalizaton of a quaternion.
	 *
	 * @ingroup tp-tests
	 */
	void test_q_normalize()
	{
		Matrix3d R = Matrix3d::Random();
		R.normalize();
		Quaterniond q = Quaterniond(1.0, 2.0, 3.0, 4.0);

		tp_quatern tq;
		copy_quaternion(q, tq);

		check_equal(q, tq);

		q.normalize();
		normalize_quaternion(tq);

		check_equal(q, tq);
	}

	/** Tests general quaternion multiplication.
	 *
	 * @ingroup tp-tests
	 */
	void test_mult_q_q()
	{
		Matrix4d Qv = Matrix4d::Random();
		Matrix4d Qv1 = Matrix4d::Random();

		Quaterniond q = Quaterniond(Qv(0), Qv(1), Qv(2), Qv(3));
		Quaterniond q1 = Quaterniond(Qv1(0), Qv1(1), Qv1(2), Qv1(3));

		tp_quatern tq, tq1;
		copy_quaternion(q, tq);
		copy_quaternion(q1, tq1);

		Quaterniond q2 = q * q1;

		tp_quatern tq2;
		mult_quatern_quatern(tq2, tq, tq1);
		check_equal(q2, tq2);

		q2 = q1 * q;
		mult_quatern_quatern(tq2, tq1, tq);
		check_equal(q2, tq2);
	}

};
