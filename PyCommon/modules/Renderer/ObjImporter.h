#ifndef _OBJ_IMPORTER_H_
#define _OBJ_IMPORTER_H_

#include <vector>

class ObjImporter
{
public:
    std::vector<float> data_tri;
    std::vector<float> data_quad;
    std::vector<float> data_penta;

    void import_obj(char* filename, float scale);
    void draw();
};

#endif  // _OBJ_IMPORTER_H_