#include <itomp_nlp/renderer/rendering_shape.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingShape::RenderingShape(Renderer* renderer)
    : GLBase(renderer)
    , material_(0)
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
    
void RenderingShape::draw(ShaderProgram* shader)
{
}

float RenderingShape::getAlpha()
{
    if (material_ == 0)
        return 1.f;

    else
        return material_->getAlpha();
}

}
