#include <itomp_nlp/shape/obb.h>

#include <itomp_nlp/shape/aabb.h>


namespace itomp
{

OBB::OBB(double x, double y, double z)
    : Shape()
    , size_(x, y, z)
{
}

OBB::OBB(double x, double y, double z, const Eigen::Affine3d& transform)
    : Shape(transform)
    , size_(x, y, z)
{
}

OBB::OBB(const AABB& aabb)
    : Shape()
{
    size_ = aabb.getUpper() - aabb.getLower();
    transform_.translate( (aabb.getLower() + aabb.getUpper()) / 2. );
}

OBB::OBB(const Eigen::Vector3d& size)
    : Shape()
    , size_(size)
{
}

OBB::OBB(const Eigen::Vector3d& size, const Eigen::Affine3d& transform)
    : Shape(transform)
    , size_(size)
{
}

}
