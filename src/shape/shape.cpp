#include <itomp_nlp/shape/shape.h>


namespace itomp
{

Shape::Shape()
{
    transform_.setIdentity();
}

Shape::Shape(const Eigen::Affine3d& transform)
    : transform_(transform)
{
}

Shape::~Shape()
{
}

void Shape::applyTransform(const Eigen::Affine3d& transform)
{
    transform_ = transform * transform_;
}

}
