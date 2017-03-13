#include <itomp_nlp/renderer/rendering_shape.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingShape::RenderingShape(Renderer* renderer)
    : GLBase(renderer)
{
    transform_.setIdentity();
    renderer->addShape(this);
}

RenderingShape::~RenderingShape()
{
    renderer_->deleteShape(this);
}

void RenderingShape::setTransform(const Eigen::Matrix4f& transform)
{
    transform_ = transform;
}

void RenderingShape::setTransform(const Eigen::Affine3d& transform)
{
    transform_ = transform.cast<float>().matrix();
}
    
void RenderingShape::draw(LightShader* shader)
{
}

void RenderingShape::draw(ColorShader* shader)
{
}

}
