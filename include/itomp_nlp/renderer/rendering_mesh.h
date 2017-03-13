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

    inline void setTransformation(const Eigen::Affine3f& transformation)
    {
        transformation_ = transformation.matrix();
    }

    inline void setTransformation(const Eigen::Matrix4f& transformation)
    {
        transformation_ = transformation;
    }

private:
    
    std::string filename_;

    void initializeBuffers();

    Material* material_;

    Eigen::Matrix4f transformation_;

    GLuint vao_;
    std::vector<GLuint> vbos_;

    int num_triangles_;
};

}


#endif // ITOMP_SHAPE_MESH_H