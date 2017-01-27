#include <itomp_nlp/shape/aabb.h>


namespace itomp
{

AABB::AABB()
    : Shape()
    , lower_(Eigen::Vector3d::Zero())
    , upper_(Eigen::Vector3d::Zero())
{
}

AABB::AABB(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper)
    : Shape()
    , lower_(lower)
    , upper_(upper)
{
}

AABB AABB::merge(const AABB& aabb)
{
    Eigen::Vector3d lower;
    Eigen::Vector3d upper;

    lower(0) = std::min(lower_(0), aabb.lower_(0));
    lower(1) = std::min(lower_(1), aabb.lower_(1));
    lower(2) = std::min(lower_(2), aabb.lower_(2));

    upper(0) = std::max(upper_(0), aabb.upper_(0));
    upper(1) = std::max(upper_(1), aabb.upper_(1));
    upper(2) = std::max(upper_(2), aabb.upper_(2));

    return AABB(lower, upper);
}

void AABB::translate(const Eigen::Vector3d& t)
{
    lower_ += t;
    upper_ += t;
}

}
