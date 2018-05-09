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

    std::vector<float> m_vVertices;
    std::vector<float> m_vNormal;
    std::vector<float> m_vTexture;
    std::vector<int> m_iTriIndexVertices;
    std::vector<int> m_iTriIndexNormal;
    std::vector<int> m_iTriIndexTexture;
    std::vector<int> m_iQuadIndexVertices;
    std::vector<int> m_iQuadIndexNormal;
    std::vector<int> m_iQuadIndexTexture;
    std::vector<int> m_iPentaIndexVertices;
    std::vector<int> m_iPentaIndexNormal;
    std::vector<int> m_iPentaIndexTexture;
    std::vector<int> m_vNumIndex;

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
                    m_iTriIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    m_iTriIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    m_iTriIndexNormal.push_back(atoi(segment_string[2].c_str())-1);
                }
                else if(line_string.size() == 5)
                {
                    m_iQuadIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    m_iQuadIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    m_iQuadIndexNormal.push_back(atoi(segment_string[2].c_str())-1);
                }
                else if(line_string.size() == 6)
                {
                    m_iPentaIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    m_iPentaIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    m_iPentaIndexNormal.push_back(atoi(segment_string[2].c_str())-1);
                }
                else
                {
                    //std::cout << "exception " << line_string.size() << " " <<std::string(buf)<<std::endl;
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

    for(int i=0; i<m_iTriIndexVertices.size(); i++)
    {
        data_tri.push_back(m_vTexture[2*m_iTriIndexTexture[i] + 0]);
        data_tri.push_back(m_vTexture[2*m_iTriIndexTexture[i] + 1]);
        data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 0]);
        data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 1]);
        data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 2]);
        data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 0]);
        data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 1]);
        data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 2]);
    }
    for(int i=0; i<m_iQuadIndexVertices.size(); i++)
    {
        data_quad.push_back(m_vTexture[2*m_iQuadIndexTexture[i] + 0]);
        data_quad.push_back(m_vTexture[2*m_iQuadIndexTexture[i] + 1]);
        data_quad.push_back(m_vNormal[3*m_iQuadIndexNormal[i] + 0]);
        data_quad.push_back(m_vNormal[3*m_iQuadIndexNormal[i] + 1]);
        data_quad.push_back(m_vNormal[3*m_iQuadIndexNormal[i] + 2]);
        data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 0]);
        data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 1]);
        data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 2]);
    }
    for(int i=0; i<m_iPentaIndexVertices.size(); i++)
    {
        data_penta.push_back(m_vTexture[2*m_iPentaIndexTexture[i] + 0]);
        data_penta.push_back(m_vTexture[2*m_iPentaIndexTexture[i] + 1]);
        data_penta.push_back(m_vNormal[3*m_iPentaIndexNormal[i] + 0]);
        data_penta.push_back(m_vNormal[3*m_iPentaIndexNormal[i] + 1]);
        data_penta.push_back(m_vNormal[3*m_iPentaIndexNormal[i] + 2]);
        data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 0]);
        data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 1]);
        data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 2]);
    }
}

void ObjImporter::draw()
{
    if(data_tri.size() > 0)
    {
        glInterleavedArrays(GL_T2F_N3F_V3F, 0, data_tri.data());
        glDrawArrays(GL_TRIANGLES, 0, data_tri.size()/8);
    }
    if(data_quad.size() > 0)
    {
        glInterleavedArrays(GL_T2F_N3F_V3F, 0, data_quad.data());
        glDrawArrays(GL_QUADS, 0, data_quad.size()/8);
    }
    if(data_penta.size() > 0)
    {
        glInterleavedArrays(GL_T2F_N3F_V3F, 0, data_penta.data());
        glDrawArrays(GL_POLYGON, 0, data_penta.size()/8);
    }
}