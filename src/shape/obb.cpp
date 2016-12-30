#include <itomp_nlp/shape/obb.h>


namespace itomp_shape
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
