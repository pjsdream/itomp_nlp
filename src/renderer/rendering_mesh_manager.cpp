#include <itomp_nlp/renderer/rendering_mesh_manager.h>


namespace itomp
{

std::unordered_map<std::string, RenderingMesh*> RenderingMeshManager::meshes_;

RenderingMesh* RenderingMeshManager::getMesh(Renderer* renderer, const std::string& filename)
{
    std::unordered_map<std::string, RenderingMesh*>::iterator it = meshes_.find(filename);
    if (it != meshes_.end())
        return it->second;

    RenderingMesh* mesh = new RenderingMesh(renderer, filename);
    meshes_[filename] = mesh;
    return mesh;
}

}
