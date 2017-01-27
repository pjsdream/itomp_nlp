#include <itomp_nlp/shape/capsule.h>


namespace itomp
{

Capsule::Capsule()
    : Capsule(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.)
{
}

Capsule::Capsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d)
    : Shape()
{
    setCapsule(p, q, d);
}

void Capsule::setCapsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d)
{
    p_[0] = p;
    p_[1] = q;
    d_ = d;
}

}
