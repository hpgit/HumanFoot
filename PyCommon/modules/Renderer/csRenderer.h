#ifndef _OBJ_IMPORTER_H_
#define _OBJ_IMPORTER_H_

#include <vector>

#ifdef WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

class ObjImporter
{
public:
    ~ObjImporter();
    std::vector<float> data_tri;
    std::vector<float> data_quad;
    std::vector<float> data_penta;
    GLsizei vbo_count;
    GLuint vboID[3];

    void import_obj(char* filename, float scale);
    void init();
    void draw();
};

class RenderContext
{
private:
    std::vector<float> data_capsule;
    // std::vector<float> data_cylinder;
public:
    RenderContext();
    void drawBox(int polygon_style);
    void drawCapsule(float radius, float height);
    void drawCylinder(float radius, float height);
    void drawHalfSphere(float radius);
    void drawSphere(float radius);
};

#endif  // _OBJ_IMPORTER_H_