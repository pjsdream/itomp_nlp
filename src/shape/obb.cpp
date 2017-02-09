#include <itomp_nlp/shape/obb.h>

#include <itomp_nlp/shape/aabb.h>
#include <itomp_nlp/shape/capsule2.h>


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

double OBB::getPenetrationDepth(Shape* shape) const
{
    OBB* obb = dynamic_cast<OBB*>(shape);
    if (obb != 0)
        return getPenetrationDepth(obb);

    Capsule2* capsule = dynamic_cast<Capsule2*>(shape);
    if (capsule != 0)
        return getPenetrationDepth(capsule);

    return 0.;
}

double OBB::getPenetrationDepth(OBB* obb) const
{
    return 0.;
}

double OBB::getPenetrationDepth(Capsule2* capsule) const
{
    static const int num_interpolations = 2;

    const Eigen::Vector3d p = inverse_transform_ * capsule->getP();
    const Eigen::Vector3d q = inverse_transform_ * capsule->getQ();
    const double rp = capsule->getRp();
    const double rq = capsule->getRq();

    double min_dist = 0.;

    for (int i=0; i<num_interpolations; i++)
    {
        const double t = (double)i / (num_interpolations - 1);
        const Eigen::Vector3d v = (1-t) * p + t * q;
        const double r = (1-t) * rp + t * rq;

        const double dist = distanceToPointNoTransform(v) - r;
        if (min_dist > dist)
            min_dist = dist;
    }

    return -min_dist;
}

double OBB::distanceToPointNoTransform(const Eigen::Vector3d& p) const
{
    if (p(0) < 0 || p(1) < 0 || p(2) < 0)
        return distanceToPointNoTransform(Eigen::Vector3d(std::abs(p(0)), std::abs(p(1)), std::abs(p(2))));

    const Eigen::Vector3d b = size_ / 2.;
    if (p(0) < b(0))
    {
        if (p(1) < b(1))
        {
            if (p(2) < b(2))
                return - std::min(std::min(b(0) - p(0), b(1) - p(1)), b(2) - p(2));

            return p(2) - b(2);
        }

        else
        {
            if (p(2) < b(2))
                return p(1) - b(1);

            return Eigen::Vector2d(p(1) - b(1), p(2) - b(2)).norm();
        }
    }

    else
    {
        if (p(1) < b(1))
        {
            if (p(2) < b(2))
                return p(0) - b(0);

            return Eigen::Vector2d(p(0) - b(0), p(2) - b(2)).norm();
        }

        else
        {
            if (p(2) < b(2))
                return Eigen::Vector2d(p(0) - b(0), p(1) - b(1)).norm();

            return (p-b).norm();
        }
    }
}

}
