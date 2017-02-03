#ifndef ITOMP_RENDERER_MESH_H
#define ITOMP_RENDERER_MESH_H


#include <itomp_nlp/renderer/rendering_shape.h>

#include <itomp_nlp/shape/mesh.h>


namespace itomp
{

class RenderingMesh : public RenderingShape
{
public:

    RenderingMesh(Renderer* renderer, Mesh* mesh);
    RenderingMesh(Renderer* renderer, const std::string& filename);

    virtual void draw(LightShader* shader);

private:
    
};

}


#endif // ITOMP_SHAPE_MESH_H