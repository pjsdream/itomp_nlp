#ifndef ITOMP_RENDERER_MESH_H
#define ITOMP_RENDERER_MESH_H


#include <itomp_nlp/renderer/rendering_shape.h>


namespace itomp
{

class RenderingMesh : public RenderingShape
{
public:

    RenderingMesh(Renderer* renderer, const std::string& filename);

private:
};

}


#endif // ITOMP_SHAPE_MESH_H