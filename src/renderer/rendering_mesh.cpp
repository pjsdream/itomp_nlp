#include <itomp_nlp/renderer/rendering_mesh.h>

// dae importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


namespace itomp
{

RenderingMesh::RenderingMesh(Renderer* renderer, Mesh* mesh)
    : RenderingShape(renderer)
{
    // TODO
}

void RenderingMesh::draw(LightShader* shader)
{
}

}
