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

    // requisite: makeCurrent before calling constructor
    RenderingCapsule(Renderer* renderer);

    virtual void draw(LightShader* shader);
    
    void setCapsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d);

private:
    
    static const int max_depth_ = 3;
    static AlignedVector<Eigen::Vector3f> normals_;
    static std::vector<int> triangles_;
    static int num_elements_;

    static void initializeNormals();
    static void generateTriangles(int depth, const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2);
    static int insertNormal(const Eigen::Vector3f& p);

    void initializeBuffers();
    void updateBuffers();

    GLuint vao_;
    std::vector<GLuint> vbos_;

    Eigen::Vector3f p_;
    Eigen::Vector3f q_;
    double d_;
};

}


#endif // ITOMP_RENDERER_CAPSULE_H