#ifndef _PYVPMATERIAL_H_
#define _PYVPMATERIAL_H_

#include <VP/vpDataType.h>

class pyVpMaterial : public vpMaterial
{
public:
	pyVpMaterial& 			self();

	/*!
		get a density of the material.
	*/
	scalar					 GetDensity_py(void);

	/*!
		set a density of the material.
	*/
	void					 SetDensity_py(scalar);

	/*!
		get a restitution parameter of the material.
	*/
	scalar					 GetRestitution_py(void);

	/*!
		set a restitution parameter of the material.
		\param e If 1, perfectly elastic. If 0, perfectly plastic.
	*/
	void					 SetRestitution_py(scalar e);

	/*!
		get a static friction parameter of the material.
	*/
	scalar					 GetStaticFriction_py(void);

	/*!
		set a static friction parameter of the material.
	*/
	void					 SetStaticFriction_py(scalar);

	/*!
		get a dynamic friction parameter of the material.
	*/
	scalar					 GetDynamicFriction_py(void);

	/*!
		set a dynamic friction parameter of the material.
	*/
	void					 SetDynamicFriction_py(scalar);

	/*!
		get a spinning friction parameter of the material.
	*/
	scalar					 GetSpinningFriction_py(void);

	/*!
		set a spinning friction parameter of the material.
	*/
	void					 SetSpinningFriction_py(scalar);

	/*!
		get a default material used for bodies which do not have their own materials.
	*/
	static vpMaterial&	GetDefaultMaterial_py(void);
};

#endif // _PYVPMATERIAL_H_
