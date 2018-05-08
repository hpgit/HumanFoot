#ifdef WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <vector>
#include <VP/vphysics.h>

const int POLYGON_LINE = 0;
const int POLYGON_FILL = 1;

const int RENDER_OBJECT = 0;
const int RENDER_SHADOW = 1;
const int RENDER_REFLECTION = 2;

class VpModel;

class GeomState
{
public:
    std::vector<int> body_id;
    std::vector<SE3> frames;
    std::vector<Vec3> color;
    std::vector<vpGeom*> geoms;
    // std::vector<char> types;
	// std::vector<Vec3> sizes;
};

class VpModelRenderer
{
private:
	VpModel* _pModel;
	GLubyte _color[3];
	int _polygonStyle;
	double _lineWidth;

	int max_frame;
	std::vector<GeomState> saved_state;
	GeomState initial_state;

public:	// expose to python
	VpModelRenderer(VpModel* pModel, const boost::python::tuple& color, int polygonStyle = POLYGON_FILL, double lineWidth = 1.);
	void render(int renderType=RENDER_OBJECT);
	void renderFrame(int frame, int renderType=RENDER_OBJECT);
	void renderState(const GeomState& state, int renderType=RENDER_OBJECT);
	void getState(GeomState& state);
	void saveState();
	int get_max_saved_frame();
};
