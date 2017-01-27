#ifndef ITOMP_SHAPE_MESH_LOADER_H
#define ITOMP_SHAPE_MESH_LOADER_H


#include <itomp_nlp/shape/mesh.h>


namespace itomp
{

class MeshLoader
{
public:

    static Mesh* loadMeshFile(const std::string& filename);

private:
};

}


#endif // ITOMP_SHAPE_MESH_LOADER_H