%module pyVirtualPhysics
%{
#define SWIG_FILE_WITH_INIT
%}
%include "numpy.i"
%init %{
import_array(); // This is essential. We will get a crash in Python without it.
%}

%apply ( double* ARGOUT_ARRAY1,int DIM1) {(double* Massout,int n)}

%include "typemaps.i"
%include "std_pair.i"
%include "std_list.i"

%{
#include <VP/LieGroup.h>
#include <VP/vpMaterial.h>
#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpWJoint.h>
#include <VP/vpRJoint.h>
#include <VP/vpPJoint.h>
#include <VP/vpUJoint.h>
#include <VP/vpSJoint.h>
#include <VP/vpBJoint.h>
#include <VP/vp1DOFJoint.h>
#include <VP/vpSpring.h>
#include <VP/vpBody.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpTimer.h>
#include <VP/vpDataType.h>
%}

%include <VP/LieGroup.h>
%include <VP/vpMaterial.h>
%include <VP/vpGeom.h>
%include <VP/vpBody.h>
%include <VP/vpJoint.h>
%include <VP/vpWJoint.h>
%include <VP/vpRJoint.h>
%include <VP/vpPJoint.h>
%include <VP/vpUJoint.h>
%include <VP/vpSJoint.h>
%include <VP/vpBJoint.h>
%include <VP/vp1DOFJoint.h>
%include <VP/vpSpring.h>
%include <VP/vpSystem.h>
%include <VP/vpWorld.h>
%include <VP/vpTimer.h>
%include <VP/vpDataType.h>
%extend Axis{
	scalar __getitem__(int idx)
	{
		return $self->operator [](idx);
	}
}
%extend SE3{
	scalar __getitem__(int idx)
	{
		return $self->operator [](idx);
	}
}
%extend se3{
	scalar __getitem__(int idx)
	{
		return $self->operator [](idx);
	}
}
%extend dse3{
	scalar __getitem__(int idx)
	{
		return $self->operator [](idx);
	}
}
%extend Inertia{
	scalar __getitem__(int idx)
	{
		return $self->operator [](idx);
	}
}


namespace VP
{
	enum HD_TYPE
	{
		KINEMATIC,			/*!< find torque for a given acceleration */
		DYNAMIC				/*!< find acceleration for a given torque */
	};
}


%extend Vec3 {
  double __getitem__(unsigned int i) {
    return (*($self))[i];
  }
}
