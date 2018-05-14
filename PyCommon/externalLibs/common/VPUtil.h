#ifndef _VPUTIL_H_
#define _VPUTIL_H_

#include <math.h>
#include <VP/vphysics.h>
#include "boostPythonUtil.h"

// #define MAKE_SO3 numpy::ndarray I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
// #define MAKE_SE3 numpy::ndarray I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
// #define MAKE_VEC3 numpy::ndarray I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

inline numpy::ndarray transpose_pySO3(numpy::ndarray& pyR)
{
	object pyR_copy = pyR.copy();

	pyR[make_tuple(0,1)] = pyR_copy[make_tuple(1,0)]; pyR[make_tuple(0,2)] = pyR_copy[make_tuple(2,0)]; 
	pyR[make_tuple(1,0)] = pyR_copy[make_tuple(0,1)]; pyR[make_tuple(1,2)] = pyR_copy[make_tuple(2,1)];
	pyR[make_tuple(2,0)] = pyR_copy[make_tuple(0,2)]; pyR[make_tuple(2,1)] = pyR_copy[make_tuple(1,2)];

	return pyR;
}

void make_pyVec3(object &pyV)
{
	numpy::ndarray O = numpy::array( make_tuple(0.,0.,0.) );
	pyV = O.copy();
}

void make_pyse3(object &pyV)
{
	numpy::ndarray O = numpy::array( make_tuple(0., 0., 0., 0.,0.,0.) );
	pyV = O.copy();
}

void make_pydse3(object &pyV)
{
	numpy::ndarray O = numpy::array( make_tuple(0.,0.,0.,0.,0.,0.) );
	pyV = O.copy();
}

void make_pyInertia(object &pyV)
{
	numpy::ndarray O = numpy::array( make_tuple(0., 0., 0., 0., 0., 0., 0., 0.,0.,0.) );
	pyV = O.copy();
}

void make_pySE3(object &pyT)
{
	numpy::ndarray I = numpy::array( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
	pyT = I.copy();
}

void make_pySO3(object &pyR)
{
	numpy::ndarray I = numpy::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	pyR = I.copy();
}

inline bool checkPyVlen(const object& pyV, int _len)
{
    const tuple	&_shape = extract<tuple>(pyV.attr("shape"));
    return (  
    		(
    		 (XI(pyV.attr("ndim")) == 1) &&
    		 (boost::python::len(pyV) == _len)
    		)
        || 
        	(
        	 (XI(pyV.attr("ndim")) == 2) &&
        	 (XI(_shape[0]) + XI(_shape[1]) == _len+1) &&
        	 (XI(_shape[0]) == _len || XI(_shape[1]) == _len)
        	)
        );

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
	return numpy::array(make_tuple(V[0], V[1], V[2]));
}
inline object Axis_2_pyVec3(const Axis& V)
{
	return numpy::array(make_tuple(V[0], V[1], V[2]));
}
inline void Vec3_2_pyVec3(const Vec3& V, object& pyV)
{
	pyV[0] = V[0]; pyV[1] = V[1]; pyV[2] = V[2];
}

inline se3 pyVec6_2_se3(const object& pyV)
{
	return se3( XD(pyV[0]), XD(pyV[1]), XD(pyV[2]), XD(pyV[3]), XD(pyV[4]), XD(pyV[5]));
}
inline dse3 pyVec6_2_dse3(const object& pyV)
{
	return dse3( XD(pyV[0]), XD(pyV[1]), XD(pyV[2]), XD(pyV[3]), XD(pyV[4]), XD(pyV[5]));
}
inline void se3_2_pyVec6(const se3& V, object& pyV)
{
	pyV[0] = V[0]; pyV[1] = V[1]; pyV[2] = V[2]; pyV[3] = V[3]; pyV[4] = V[4]; pyV[5] = V[5];
}
inline void dse3_2_pyVec6(const dse3& V, object& pyV)
{
	pyV[0] = V[0]; pyV[1] = V[1]; pyV[2] = V[2]; pyV[3] = V[3]; pyV[4] = V[4]; pyV[5] = V[5];
}


//		| T[0]	T[3]	T[6]	T[ 9] |
//		| T[1]	T[4]	T[7]	T[10] |
//		| T[2]	T[5]	T[8]	T[11] |
inline SE3 pySO3_2_SE3(const object& pyR)
{
	return SE3(XD(pyR[0][0]), XD(pyR[1][0]), XD(pyR[2][0]), 
				XD(pyR[0][1]), XD(pyR[1][1]), XD(pyR[2][1]), 
				XD(pyR[0][2]), XD(pyR[1][2]), XD(pyR[2][2]));
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

inline SE3 pySE3_2_SE3(const object& pyT)
{
	return SE3(XD(pyT[0][0]), XD(pyT[1][0]), XD(pyT[2][0]),
				XD(pyT[0][1]), XD(pyT[1][1]), XD(pyT[2][1]),
				XD(pyT[0][2]), XD(pyT[1][2]), XD(pyT[2][2]),
				XD(pyT[0][3]), XD(pyT[1][3]), XD(pyT[2][3]));
}
inline void pySE3_2_SE3(const object& pyT, SE3& T)
{
	T[0]  = XD(pyT[0][0]);
	T[1]  = XD(pyT[1][0]);
	T[2]  = XD(pyT[2][0]);
	T[3]  = XD(pyT[0][1]);
	T[4]  = XD(pyT[1][1]);
	T[5]  = XD(pyT[2][1]);
	T[6]  = XD(pyT[0][2]);
	T[7]  = XD(pyT[1][2]);
	T[8]  = XD(pyT[2][2]);
	T[9]  = XD(pyT[0][3]);
	T[10] = XD(pyT[1][3]);
	T[11] = XD(pyT[2][3]);
}
inline void SE3_2_pySO3(const SE3& T, object& pyR)
{
	pyR[make_tuple(0,0)] = T[0]; pyR[make_tuple(0,1)] = T[3]; pyR[make_tuple(0,2)] = T[6];
	pyR[make_tuple(1,0)] = T[1]; pyR[make_tuple(1,1)] = T[4]; pyR[make_tuple(1,2)] = T[7];
	pyR[make_tuple(2,0)] = T[2]; pyR[make_tuple(2,1)] = T[5]; pyR[make_tuple(2,2)] = T[8];
}
inline numpy::ndarray SE3_2_pySO3(const SE3& T)
{
	return numpy::array(make_tuple(
							make_tuple(T[0], T[3], T[6]), 
							make_tuple(T[1], T[4], T[7]),
							make_tuple(T[2], T[5], T[8])
							));
}

inline void SE3_2_pySE3(const SE3& T, object& pyT)
{
	SE3_2_pySO3(T, pyT);
	pyT[make_tuple(0,3)] = T[9];
	pyT[make_tuple(1,3)] = T[10];
	pyT[make_tuple(2,3)] = T[11];
	pyT[make_tuple(3,0)] = 0.;
	pyT[make_tuple(3,1)] = 0.;
	pyT[make_tuple(3,2)] = 0.;
	pyT[make_tuple(3,3)] = 1.;
}

inline SE3 slerp(const SE3& R1, const SE3& R2, double t)
{
    //return numpy.dot(R1, exp(t * logSO3( numpy.dot(R1.transpose(), R2) )))	
	return R1 * Exp(t * Log(Inv(R1) * R2));
}

SE3 getSE3FromVectors(const Vec3& vec1, const Vec3& vec2)
{
	Vec3 v1 = Normalize(vec1);
	Vec3 v2 = Normalize(vec2);

	Vec3 rot_axis = Normalize(Cross(v1, v2));
	scalar inner = Inner(v1, v2);
	scalar theta = acos(inner);

//	if( rot_axis[0]==0 && rot_axis[1]==0 && rot_axis[2]==0)
//		rot_axis = Vec3(0,1,0);
	if( Norm(rot_axis) < LIE_EPS )
		rot_axis = Vec3(0,1,0);
	else if( inner < -1.0 + LIE_EPS)
	{
//	    Vec3 rand_vec(rand(), rand(), rand());
//	    rot_axis = Normalize(Cross(v1, Normalize(rand_vec)));
        rot_axis = Vec3(0., 1., 0.);
        inner = -1.;
        theta = M_PI;
	}



	scalar x = rot_axis[0];
	scalar y = rot_axis[1];
	scalar z = rot_axis[2];

	scalar c = inner;
	scalar s = sin(theta);

	SE3 R(c + (1.0-c)*x*x,    (1.0-c)*x*y - s*z,    (1-c)*x*z + s*y,
        (1.0-c)*x*y + s*z,    c + (1.0-c)*y*y,    (1.0-c)*y*z - s*x,
        (1.0-c)*z*x - s*y,    (1.0-c)*z*y + s*x,    c + (1.0-c)*z*z);

	return Inv(R);
}

inline Axis Vec3_2_Axis(const Vec3& v)
{
    return Axis(v[0], v[1], v[2]);
}

#endif //_VPUTIL_H_
