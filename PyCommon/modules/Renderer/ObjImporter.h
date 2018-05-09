#ifndef _OBJ_IMPORTER_H_
#define _OBJ_IMPORTER_H_

#include <vector>

class ObjImporter
{
public:
    std::vector<float> m_vVertices;
    std::vector<float> m_vNormal;
    std::vector<float> m_vTexture;
    std::vector<int> m_iTriIndexVertices;
    std::vector<int> m_iTriIndexNormal;
    std::vector<int> m_iTriIndexTexture;
    std::vector<int> m_iQuadIndexVertices;
    std::vector<int> m_iQuadIndexNormal;
    std::vector<int> m_iQuadIndexTexture;
    std::vector<int> m_vNumIndex;

    void import_obj(char* filename, float scale);
    void draw();
};

#endif  // _OBJ_IMPORTER_H_