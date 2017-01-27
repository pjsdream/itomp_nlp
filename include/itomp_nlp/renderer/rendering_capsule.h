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
public:

    RenderingCapsule(Renderer* renderer);
    
    void setCapsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d);

private:

};

}


#endif // ITOMP_RENDERER_CAPSULE_H