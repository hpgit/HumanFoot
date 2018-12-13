#pragma once

#ifdef WIN32
#include <windows.h> 
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class IMSModel;
class IMSModelRenderer
{
public:
	IMSModel* _pModel;
	GLubyte _color[3];
	bool _drawParticles;
	dict _colorMap;
	void _IMSModelRenderer(IMSModel* pModel, const boost::python::tuple& color, bool drawParticles, const dict& colorMap);

public:	// expose to python
	IMSModelRenderer(IMSModel* pModel);
	IMSModelRenderer(IMSModel* pModel, const boost::python::tuple& color);
	IMSModelRenderer(IMSModel* pModel, const boost::python::tuple& color, bool drawParticles);
	IMSModelRenderer(IMSModel* pModel, const boost::python::tuple& color, const dict& colorMap);
	void render();
};
