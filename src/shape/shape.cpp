#include <itomp_nlp/shape/shape.h>


namespace itomp
{

Shape::Shape()
{
    transform_.setIdentity();
    inverse_transform_ = transform_.inverse();
}

Shape::Shape(const Eigen::Affine3d& transform)
    : transform_(transform)
{
    inverse_transform_ = transform_.inverse();
}

Shape::~Shape()
{
}

double Shape::getPenetrationDepth(Shape* shape) const
{
    return 0.;
}

void Shape::applyTransform(const Eigen::Affine3d& transform)
{
    transform_ = transform * transform_;
    inverse_transform_ = transform_.inverse();
}

}
