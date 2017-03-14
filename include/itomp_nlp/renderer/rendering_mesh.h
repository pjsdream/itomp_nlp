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
    ~RenderingMesh();

    virtual void draw(LightShader* shader);
    virtual void draw(ShadowmapShader* shader);

private:
    
    std::string filename_;

    void initializeBuffers();

    GLuint vao_;
    std::vector<GLuint> vbos_;

    bool has_tex_coords_;

    int num_triangles_;
};

}


#endif // ITOMP_SHAPE_MESH_H