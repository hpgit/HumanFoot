#include "stdafx.h"

#include "csRenderer.h"


#include <fstream>
#include <sstream>
#include <iostream>

BOOST_PYTHON_MODULE(csRenderer)
{
    class_<ObjImporter>("ObjImporter")
        .def("import_obj", &ObjImporter::import_obj)
        .def("init", &ObjImporter::init)
        .def("draw", &ObjImporter::draw)
        ;

    class_<RenderContext>("RenderContext")
        .def("drawBox", &RenderContext::drawBox)
//        .def("drawCapsule", &RenderContext::drawCapsule)
        ;
}

ObjImporter::~ObjImporter()
{
    if (this->vbo_count > 0)
        glDeleteBuffers(this->vbo_count, this->vboID);
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
                for(std::vector<std::string>::size_type i=1; i<line_string.size(); i++)
                    m_vNormal.push_back(atof(line_string[i].c_str()));

            else if(buf[1] == 't')
                for(std::vector<std::string>::size_type i=1; i<line_string.size(); i++)
                    m_vTexture.push_back(atof(line_string[i].c_str()));

            else
                for(std::vector<std::string>::size_type i=1; i<line_string.size(); i++)
                    m_vVertices.push_back(scale * atof(line_string[i].c_str()));
        }
        else if(buf[0] == 'f')
        {
            //faces
            while(getline(line, s, ' '))
                line_string.push_back(s);

            for(std::vector<std::string>::size_type i=1; i<line_string.size(); i++)
            {
                std::istringstream segment(line_string[i].c_str());
                std::vector<std::string> segment_string;
                while(getline(segment, s, '/'))
                    segment_string.push_back(s);

                if(line_string.size() == 4)
                {
                    m_iTriIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    if(segment_string.size() > 1)
                        m_iTriIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    if(segment_string.size() > 2)
                        m_iTriIndexNormal.push_back(atoi(segment_string[2].c_str())-1);
                }
                else if(line_string.size() == 5)
                {
                    m_iQuadIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    if(segment_string.size() > 1)
                        m_iQuadIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    if(segment_string.size() > 2)
                        m_iQuadIndexNormal.push_back(atoi(segment_string[2].c_str())-1);
                }
                else if(line_string.size() == 6)
                {
                    m_iPentaIndexVertices.push_back(atoi(segment_string[0].c_str())-1);
                    if(segment_string.size() > 1)
                        m_iPentaIndexTexture.push_back(atoi(segment_string[1].c_str())-1);
                    if(segment_string.size() > 2)
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

    tri_texture_on = (m_iTriIndexTexture.size() > 0);
    tri_normal_on = (m_iTriIndexNormal.size() > 0);
    quad_texture_on = (m_iQuadIndexTexture.size() > 0);
    quad_normal_on = (m_iQuadIndexNormal.size() > 0);
    penta_texture_on = (m_iPentaIndexTexture.size() > 0);
    penta_normal_on = (m_iPentaIndexNormal.size() > 0);

    // temporary
    tri_texture_on = false;
    quad_texture_on = false;
    penta_texture_on = false;

    for(std::vector<int>::size_type i=0; i<m_iTriIndexVertices.size(); i++)
    {
        if (tri_texture_on) {
            this->data_tri.push_back(m_vTexture[2 * m_iTriIndexTexture[i] + 0]);
            this->data_tri.push_back(m_vTexture[2 * m_iTriIndexTexture[i] + 1]);
        }
        if (tri_normal_on){
            this->data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 0]);
            this->data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 1]);
            this->data_tri.push_back(m_vNormal[3*m_iTriIndexNormal[i] + 2]);
        }

        this->data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 0]);
        this->data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 1]);
        this->data_tri.push_back(m_vVertices[3*m_iTriIndexVertices[i] + 2]);
    }
    for(std::vector<int>::size_type i=0; i<m_iQuadIndexVertices.size(); i++)
    {
        if (quad_texture_on) {
            this->data_quad.push_back(m_vTexture[2 * m_iQuadIndexTexture[i] + 0]);
            this->data_quad.push_back(m_vTexture[2 * m_iQuadIndexTexture[i] + 1]);
        }
        if (quad_normal_on) {
            this->data_quad.push_back(m_vNormal[3 * m_iQuadIndexNormal[i] + 0]);
            this->data_quad.push_back(m_vNormal[3 * m_iQuadIndexNormal[i] + 1]);
            this->data_quad.push_back(m_vNormal[3 * m_iQuadIndexNormal[i] + 2]);
        }
        this->data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 0]);
        this->data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 1]);
        this->data_quad.push_back(m_vVertices[3*m_iQuadIndexVertices[i] + 2]);
    }
    for(std::vector<int>::size_type i=0; i<m_iPentaIndexVertices.size(); i++)
    {
        if (penta_texture_on) {
            this->data_penta.push_back(m_vTexture[2 * m_iPentaIndexTexture[i] + 0]);
            this->data_penta.push_back(m_vTexture[2 * m_iPentaIndexTexture[i] + 1]);
        }
        if (penta_normal_on) {
            this->data_penta.push_back(m_vNormal[3 * m_iPentaIndexNormal[i] + 0]);
            this->data_penta.push_back(m_vNormal[3 * m_iPentaIndexNormal[i] + 1]);
            this->data_penta.push_back(m_vNormal[3 * m_iPentaIndexNormal[i] + 2]);
        }
        this->data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 0]);
        this->data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 1]);
        this->data_penta.push_back(m_vVertices[3*m_iPentaIndexVertices[i] + 2]);
    }

//    if (this->data_tri.size() > 0) this->vbo_count++;
//    if (this->data_quad.size() > 0) this->vbo_count++;
//    if (this->data_penta.size() > 0) this->vbo_count++;

}

#ifndef BUFFER_OFFSET
#define BUFFER_OFFSET(i) ((void*)(i))
#endif

void ObjImporter::init()
{
    glGenBuffers(this->vbo_count, this->vboID);

    if(data_tri.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[0]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(this->data_tri), this->data_tri.data(), GL_STATIC_DRAW);
    }
    if(data_quad.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(this->data_quad), this->data_quad.data(), GL_STATIC_DRAW);
    }
    if(data_penta.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[2]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(this->data_penta), this->data_penta.data(), GL_STATIC_DRAW);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ObjImporter::draw()
{
    // glInterleavedArrays example
    if(data_tri.size() > 0)
    {
        if (tri_texture_on && tri_normal_on)
        {
            glInterleavedArrays(GL_T2F_N3F_V3F, 0, this->data_tri.data());
            glDrawArrays(GL_TRIANGLES, 0, this->data_tri.size()/8);
        }
        else if (tri_texture_on)
        {
            glInterleavedArrays(GL_T2F_V3F, 0, this->data_tri.data());
            glDrawArrays(GL_TRIANGLES, 0, this->data_tri.size()/5);
        }
        else if (tri_normal_on)
        {
            glInterleavedArrays(GL_N3F_V3F, 0, this->data_tri.data());
            glDrawArrays(GL_TRIANGLES, 0, this->data_tri.size()/6);
        }
        else
        {
            glInterleavedArrays(GL_V3F, 0, this->data_tri.data());
            glDrawArrays(GL_TRIANGLES, 0, this->data_tri.size()/3);
        }
    }

    if(data_quad.size() > 0)
    {
        if (quad_texture_on && quad_normal_on)
        {
            glInterleavedArrays(GL_T2F_N3F_V3F, 0, this->data_quad.data());
            glDrawArrays(GL_QUADS, 0, this->data_quad.size()/8);
        }
        else if (quad_texture_on)
        {
            glInterleavedArrays(GL_T2F_V3F, 0, this->data_quad.data());
            glDrawArrays(GL_QUADS, 0, this->data_quad.size()/5);
        }
        else if (quad_normal_on)
        {
            glInterleavedArrays(GL_N3F_V3F, 0, this->data_quad.data());
            glDrawArrays(GL_QUADS, 0, this->data_quad.size()/6);
        }
        else
        {
            glInterleavedArrays(GL_V3F, 0, this->data_quad.data());
            glDrawArrays(GL_QUADS, 0, this->data_quad.size()/3);
        }
    }

    if(data_penta.size() > 0)
    {
        if (penta_texture_on && penta_normal_on)
        {
            glInterleavedArrays(GL_T2F_N3F_V3F, 0, this->data_penta.data());
            glDrawArrays(GL_POLYGON, 0, this->data_penta.size()/8);
        }
        else if (penta_texture_on)
        {
            glInterleavedArrays(GL_T2F_V3F, 0, this->data_penta.data());
            glDrawArrays(GL_POLYGON, 0, this->data_penta.size()/5);
        }
        else if (penta_normal_on)
        {
            glInterleavedArrays(GL_N3F_V3F, 0, this->data_penta.data());
            glDrawArrays(GL_POLYGON, 0, this->data_penta.size()/6);
        }
        else
        {
            glInterleavedArrays(GL_V3F, 0, this->data_penta.data());
            glDrawArrays(GL_POLYGON, 0, this->data_penta.size()/3);
        }
    }

/*
    if (data_tri.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[0]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(sizeof(float)*3));
        glDrawArrays(GL_TRIANGLES, 0, this->data_tri.size()/100);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    if (data_quad.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[1]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(sizeof(float)*3));
        glDrawArrays(GL_QUADS, 0, this->data_quad.size()/6);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    if (data_penta.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, this->vboID[2]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(0));
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(float)*3, BUFFER_OFFSET(sizeof(float)*3));
        glDrawArrays(GL_POLYGON, 0, this->data_penta.size()/6);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    */
}

RenderContext::RenderContext()
{

}


static float box_line_point_array[] =
        {
            -.5, -.5, -.5,
            -.5, -.5, +.5,
            +.5, -.5, -.5,
            +.5, -.5, +.5,
            -.5, +.5, -.5,
            -.5, +.5, +.5,
            +.5, +.5, -.5,
            +.5, +.5, +.5,

            +.5, -.5, -.5,
            -.5, -.5, -.5,
            +.5, +.5, -.5,
            -.5, +.5, -.5,
            +.5, -.5, +.5,
            -.5, -.5, +.5,
            +.5, +.5, +.5,
            -.5, +.5, +.5,

            -.5, -.5, -.5,
            -.5, +.5, -.5,
            -.5, -.5, +.5,
            -.5, +.5, +.5,
            +.5, -.5, +.5,
            +.5, +.5, +.5,
            +.5, -.5, -.5,
            +.5, +.5, -.5,
        };

static float box_fill_point_array[] =
        {
            -.5, -.5, -.5,
            -.5, +.5, -.5,
            +.5, +.5, -.5,
            +.5, -.5, -.5,
            -.5, -.5, +.5,
            +.5, -.5, +.5,
            +.5, +.5, +.5,
            -.5, +.5, +.5,
            -.5, -.5, -.5,
            -.5, -.5, +.5,
            -.5, +.5, +.5,
            -.5, +.5, -.5,
            +.5, -.5, -.5,
            +.5, +.5, -.5,
            +.5, +.5, +.5,
            +.5, -.5, +.5,
            -.5, -.5, -.5,
            +.5, -.5, -.5,
            +.5, -.5, +.5,
            -.5, -.5, +.5,
            -.5, +.5, -.5,
            -.5, +.5, +.5,
            +.5, +.5, +.5,
            +.5, +.5, -.5
        };

static float box_fill_normal_array[] =
        {
            0., 0., -1.,
            0., 0., -1.,
            0., 0., -1.,
            0., 0., -1.,
            0., 0., +1.,
            0., 0., +1.,
            0., 0., +1.,
            0., 0., +1.,
            -1., 0., 0.,
            -1., 0., 0.,
            -1., 0., 0.,
            -1., 0., 0.,
            +1., 0., 0.,
            +1., 0., 0.,
            +1., 0., 0.,
            +1., 0., 0.,
            0., -1., 0.,
            0., -1., 0.,
            0., -1., 0.,
            0., -1., 0.,
            0., +1., 0.,
            0., +1., 0.,
            0., +1., 0.,
            0., +1., 0.
        };

void RenderContext::drawBox(int polygon_style)
{
    if (polygon_style == 0)
        glPolygonMode(GL_FRONT, GL_LINE);
    else
        glPolygonMode(GL_FRONT, GL_FILL);
    int vertex_size = 24;
    glVertexPointer(3, GL_FLOAT, 0, box_fill_point_array);
    glNormalPointer(GL_FLOAT, 0, box_fill_normal_array);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glDrawArrays(GL_QUADS, 0, vertex_size);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glPolygonMode(GL_FRONT, GL_FILL);
}

