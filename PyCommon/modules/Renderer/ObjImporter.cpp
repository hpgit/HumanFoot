#include "stdafx.h"

#include "ObjImporter.h"

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

#include <fstream>
#include <sstream>
#include <iostream>

BOOST_PYTHON_MODULE(ObjImporter)
{
    class_<ObjImporter>("ObjImporter")
        .def("import_obj", &ObjImporter::import_obj)
        .def("draw", &ObjImporter::draw)
        ;
}

void ObjImporter::import_obj(char* filename, float scale)
{
    std::ifstream fin(filename);

    char buf[255];

    int iNumGroup = 0;
    int iNumFaceSize = 0;

    while(!fin.eof())
    {
        fin.getline(buf, 254);
        std::istringstream line(buf);
        std::vector<std::string> line_string;
        std::string s(buf);
        // std::cout << s.size() << std::endl;
        // std::cout << s << std::endl;

        if(buf[0] == '#')
        {
            //comment
        }
        else if(buf[0] == 'o')
        {
            //object name
        }
        else if(buf[0] == 'g')
        {
            //group name
        }

        else if(buf[0] == 'v')
        {
            // vertices
            while(getline(line, s, ' '))
                line_string.push_back(s);


            if(buf[1] == 'n')
                for(int i=1; i<line_string.size(); i++)
                    m_vNormal.push_back(atof(line_string[i].c_str()));

            else if(buf[1] == 't')
                for(int i=1; i<line_string.size(); i++)
                    m_vTexture.push_back(atof(line_string[i].c_str()));

            else
                for(int i=1; i<line_string.size(); i++)
                    m_vVertices.push_back(scale * atof(line_string[i].c_str()));
        }
        else if(buf[0] == 'f')
        {
            //faces
            while(getline(line, s, ' '))
                line_string.push_back(s);

            for(int i=1; i<line_string.size(); i++)
            {
                std::istringstream segment(line_string[i].c_str());
                std::vector<std::string> segment_string;
                while(getline(segment, s, '/'))
                    segment_string.push_back(s);

                if(line_string.size() == 4)
                {
                    m_iTriIndexVertices.push_back(atoi(segment_string[1].c_str()));
                    m_iTriIndexTexture.push_back(atoi(segment_string[2].c_str()));
                    m_iTriIndexNormal.push_back(atoi(segment_string[3].c_str()));
                }
                else if(line_string.size() == 5)
                {
                    m_iQuadIndexVertices.push_back(atoi(segment_string[1].c_str()));
                    m_iQuadIndexTexture.push_back(atoi(segment_string[2].c_str()));
                    m_iQuadIndexNormal.push_back(atoi(segment_string[3].c_str()));
                }
            }
        }
        else if(buf[0] == 's')
        {
            //smooth shading
        }
        else if(buf[0] == 'm')
        {
            //mtllib
        }
        else if(buf[0] == 'u')
        {
            //usemtl
        }
        else if(buf[0] == 'p')
        {
            //point
        }
        else if(buf[0] == 'l')
        {
            //line
        }
    }

    fin.close();
}

void ObjImporter::draw()
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, m_vVertices.data());
    // glNormalPointer(GL_FLOAT, 0, normal.data());

    // glEnableClientState(GL_NORMAL_ARRAY);
//    glDrawElements(GL_QUADS, m_iQuadIndexVertices.size()/4, GL_UNSIGNED_INT, m_iQuadIndexVertices.data());
    glDrawElements(GL_QUADS, 500, GL_UNSIGNED_INT, m_iQuadIndexVertices.data());
    // glDrawElements(GL_TRIANGLES, 0, VertexSize);
    // glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}