#include "stdafx.h"

#include "VPUtil.h"


SE3 getSE3FromVectors(const Vec3& vec1, const Vec3& vec2)
{
	Vec3 v1 = Normalize(vec1);
	Vec3 v2 = Normalize(vec2);

	Vec3 rot_axis = Normalize(Cross(v1, v2));
	scalar inner = Inner(v1, v2);
	scalar theta = acos(inner);

	if( rot_axis[0]==0 && rot_axis[1]==0 && rot_axis[2]==0)
		rot_axis = Vec3(0,1,0);

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
