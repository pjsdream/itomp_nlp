#ifndef ITOMP_RENDERER_KINECT_POINTS_H
#define ITOMP_RENDERER_KINECT_POINTS_H


#include <itomp_nlp/renderer/rendering_shape.h>

#include <itomp_nlp/kinect/kinect.h>


namespace itomp
{

class RenderingKinectPoints : public RenderingShape
{
public:

    RenderingKinectPoints(Renderer* renderer);
    ~RenderingKinectPoints();

    virtual void draw(ColorShader* shader);

    inline void setTransform(const Eigen::Affine3d& transform)
    {
        transform_ = transform.matrix().cast<float>();
    }
    
private:
    
    Eigen::Matrix4f transform_;
    
    Kinect* kinect_;

    void updateBuffers();

    GLuint vao_;
    GLuint vbos_[2];
};

}


#endif // ITOMP_RENDERER_KINECT_POINTS_H