#pragma once

#include <math.h>
#include <gear/gear.h>
#include "../../externalLibs/common/boostPythonUtil.h"


inline numeric::array transpose_pySO3(numeric::array& pyR)
{
	object pyR_copy = pyR.copy();

	pyR[make_tuple(0,1)] = pyR_copy[make_tuple(1,0)]; pyR[make_tuple(0,2)] = pyR_copy[make_tuple(2,0)]; 
	pyR[make_tuple(1,0)] = pyR_copy[make_tuple(0,1)]; pyR[make_tuple(1,2)] = pyR_copy[make_tuple(2,1)];
	pyR[make_tuple(2,0)] = pyR_copy[make_tuple(0,2)]; pyR[make_tuple(2,1)] = pyR_copy[make_tuple(1,2)];

	return pyR;
}

inline Vec3 pyVec3_2_Vec3(const object& pyV)
{
	return Vec3( XD(pyV[0]), XD(pyV[1]), XD(pyV[2]) );
}
inline void pyVec3_2_Vec3(const object& pyV, Vec3& V)
{
	V[0]=XD(pyV[0]); V[1]=XD(pyV[1]); V[2]=XD(pyV[2]);
}
inline object Vec3_2_pyVec3(const Vec3& V)
{
	return numeric::array(make_tuple(V[0], V[1], V[2]));
}
inline void Vec3_2_pyVec3(const Vec3& V, object& pyV)
{
	pyV[0] = V[0]; pyV[1] = V[1]; pyV[2] = V[2];
}

inline void double3_2_pyVec3(double d0,double d1,double d2, object& pyV)
{
	pyV[0] = d0; pyV[1] = d1; pyV[2] = d2;
}

//		| T[0]	T[3]	T[6]	T[ 9] |
//		| T[1]	T[4]	T[7]	T[10] |
//		| T[2]	T[5]	T[8]	T[11] |
inline SE3 pySO3_2_SE3(const object& pyR)
{
	return SE3(SO3(XD(pyR[0][0]), XD(pyR[1][0]), XD(pyR[2][0]), 
				XD(pyR[0][1]), XD(pyR[1][1]), XD(pyR[2][1]), 
				XD(pyR[0][2]), XD(pyR[1][2]), XD(pyR[2][2])));
}
inline void pySO3_2_SE3(const object& pyR, SE3& T)
{
	T[0] = XD(pyR[0][0]);
	T[3] = XD(pyR[0][1]);
	T[6] = XD(pyR[0][2]);
	T[1] = XD(pyR[1][0]);
	T[4] = XD(pyR[1][1]);
	T[7] = XD(pyR[1][2]);
	T[2] = XD(pyR[2][0]);
	T[5] = XD(pyR[2][1]);
	T[8] = XD(pyR[2][2]);
	T[9] = T[10] = T[11] = 0.;
}
inline void SE3_2_pySO3(const SE3& T, object& pyR)
{
	pyR[make_tuple(0,0)] = T[0]; pyR[make_tuple(0,1)] = T[3]; pyR[make_tuple(0,2)] = T[6];
	pyR[make_tuple(1,0)] = T[1]; pyR[make_tuple(1,1)] = T[4]; pyR[make_tuple(1,2)] = T[7];
	pyR[make_tuple(2,0)] = T[2]; pyR[make_tuple(2,1)] = T[5]; pyR[make_tuple(2,2)] = T[8]; 
}

inline void SE3_2_pySE3(const SE3& T, object& pyT)
{
	SE3_2_pySO3(T, pyT);
	pyT[make_tuple(0,3)] = T[9];
	pyT[make_tuple(1,3)] = T[10];
	pyT[make_tuple(2,3)] = T[11];
}

inline SE3 slerp(const SE3& R1, const SE3& R2, double t)
{
    //return numpy.dot(R1, exp(t * logSO3( numpy.dot(R1.transpose(), R2) )))	
	return R1 * Exp(t * Log(Inv(R1) * R2));
}

SE3 getSE3FromVectors(const Vec3& vec1, const Vec3& vec2)
{
	Vec3 v1 = vec1;
	Vec3 v2 = vec2;
	v1.Normalize();
	v2.Normalize();

	Vec3 rot_axis = Cross(v1, v2);
	rot_axis.Normalize();
	double inner = Inner(v1, v2);
	double theta = acos(inner);

	if( rot_axis[0]==0 && rot_axis[1]==0 && rot_axis[2]==0)
		rot_axis = Vec3(0,1,0);

	double x = rot_axis[0];
	double y = rot_axis[1];
	double z = rot_axis[2];

	double c = inner;
	double s = sin(theta);

	SO3 R(c + (1.0-c)*x*x,    (1.0-c)*x*y - s*z,    (1-c)*x*z + s*y,
        (1.0-c)*x*y + s*z,    c + (1.0-c)*y*y,    (1.0-c)*y*z - s*x,
        (1.0-c)*z*x - s*y,    (1.0-c)*z*y + s*x,    c + (1.0-c)*z*z);
	
	return SE3(Inv(R));
}
