#ifndef ITOMP_RENDERER_MESH_MANAGER_H
#define ITOMP_RENDERER_MESH_MANAGER_H


#include <itomp_nlp/renderer/rendering_mesh.h>

#include <unordered_map>
#include <string>


namespace itomp
{

class RenderingMeshManager
{
public:

    static RenderingMesh* getMesh(Renderer* renderer, const std::string& filename);

private:

    static std::unordered_map<std::string, RenderingMesh*> meshes_;
};

}


#endif // ITOMP_RENDERER_MESH_MANAGER_H