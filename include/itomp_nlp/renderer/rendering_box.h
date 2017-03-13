#ifndef ITOMP_RENDERER_BOX_H
#define ITOMP_RENDERER_BOX_H


#include <itomp_nlp/renderer/rendering_shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class RenderingBox : public RenderingShape
{
private:

    template <typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

public:

    RenderingBox(Renderer* renderer);
    ~RenderingBox();

    virtual void draw(LightShader* shader);
    
    void setSize(const Eigen::Vector3d& size);

private:
    
    bool need_update_buffer_;

    void addFace(AlignedVector<Eigen::Vector3f>& vertices, AlignedVector<Eigen::Vector3f>& normals, const Eigen::Vector3f& normal);
    void updateBuffers();

    GLuint vao_;
    std::vector<GLuint> vbos_;

    Eigen::Vector3d size_;
};

}


#endif // ITOMP_RENDERER_BOX_H