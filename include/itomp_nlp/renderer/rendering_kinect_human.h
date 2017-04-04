#ifndef ITOMP_RENDERER_KINECT_HUMAN_H
#define ITOMP_RENDERER_KINECT_HUMAN_H


#include <itomp_nlp/renderer/rendering_shape.h>
#include <itomp_nlp/renderer/rendering_human.h>

#include <itomp_nlp/kinect/kinect.h>

#include <set>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class RenderingKinectHuman : public RenderingShape
{
public:

    RenderingKinectHuman(Renderer* renderer);

    virtual void draw(ShaderProgram* shader);
    
private:
    
    Kinect* kinect_;

    void updateBuffers();

    std::vector<RenderingHuman*> humans_;
};

}


#endif // ITOMP_RENDERER_KINECT_H