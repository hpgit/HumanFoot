#pragma once
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>


const int POLYGON_LINE = 0;
const int POLYGON_FILL = 1;

class GearModel;

class GearModelRenderer
{
private:
	GearModel* _pModel;
	GLubyte _color[3];
	int _polygonStyle;
	double _lineWidth;

public:	// expose to python
	GearModelRenderer(GearModel* pModel, const tuple& color, int polygonStyle=POLYGON_FILL, double lineWidth=1.);
	void render();
};
