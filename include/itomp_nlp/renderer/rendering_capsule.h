#ifndef ITOMP_RENDERER_CAPSULE_H
#define ITOMP_RENDERER_CAPSULE_H


#include <itomp_nlp/renderer/rendering_shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class RenderingCapsule : public RenderingShape
{
private:

    template <typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

    template <typename T1, typename T2>
    using AlignedMap = std::map<T1, T2, std::less<T1>, 
         Eigen::aligned_allocator<std::pair<const T1, T2> > >;

public:

    RenderingCapsule(Renderer* renderer);
    ~RenderingCapsule();

    virtual void draw(ShaderProgram* shader);
    
    void setCapsule(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rq);

private:
    
    static const int num_interpolations_ = 16;
    static const int num_triangles_ = 2 * num_interpolations_ * (4 * num_interpolations_ - 4);

    void updateBuffers();

    GLuint vao_;
    std::vector<GLuint> vbos_;

    bool need_update_buffer_;
    Eigen::Vector3f p_;
    Eigen::Vector3f q_;
    double rp_;
    double rq_;
};

}


#endif // ITOMP_RENDERER_CAPSULE_H