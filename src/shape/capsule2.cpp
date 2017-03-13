#include <itomp_nlp/shape/capsule2.h>


namespace itomp
{

Capsule2::Capsule2()
    : Capsule2(Eigen::Vector3d::Zero(), 1., Eigen::Vector3d::Zero(), 1.)
{
}

Capsule2::Capsule2(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rq)
    : Shape()
{
    setCapsule(p, rp, q, rq);
}

void Capsule2::setCapsule(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rq)
{
    p_[0] = p;
    p_[1] = q;
    r_[0] = rp;
    r_[1] = rq;
}

}
