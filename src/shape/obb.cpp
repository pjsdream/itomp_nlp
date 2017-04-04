#include <itomp_nlp/shape/obb.h>

#include <itomp_nlp/shape/aabb.h>
#include <itomp_nlp/shape/capsule2.h>


namespace itomp
{

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

OBB::EigenAlignedVector<Eigen::Vector3d> OBB::getEndpoints() const
{
    static EigenAlignedVector<Eigen::Vector3d> base_endpoints = 
    {
        Eigen::Vector3d(-0.5, -0.5, -0.5),
        Eigen::Vector3d(-0.5, -0.5,  0.5),
        Eigen::Vector3d(-0.5,  0.5, -0.5),
        Eigen::Vector3d(-0.5,  0.5,  0.5),
        Eigen::Vector3d( 0.5, -0.5, -0.5),
        Eigen::Vector3d( 0.5, -0.5,  0.5),
        Eigen::Vector3d( 0.5,  0.5, -0.5),
        Eigen::Vector3d( 0.5,  0.5,  0.5),
    };
    
    EigenAlignedVector<Eigen::Vector3d> endpoints(8);

    for (int i=0; i<8; i++)
        endpoints[i] = transform_ * (size_.cwiseProduct(base_endpoints[i]));

    return endpoints;
}

double OBB::getPenetrationDepth(OBB* obb) const
{
    // half extents
    Eigen::VectorXd sizes(6);
    sizes.block(0, 0, 3, 1) = size_ / 2.;
    sizes.block(3, 0, 3, 1) = obb->size_ / 2.;

    // find normal candidates
    Eigen::Matrix<double, 3, 15> normals;
    
    normals.block(0, 0, 3, 3) = transform_.linear();
    normals.block(0, 3, 3, 3) = obb->getTransform().linear();
    
    Eigen::Matrix3d outer;
    outer(0, 0) = 0.;
    outer(1, 1) = 0.;
    outer(2, 2) = 0.;
    for (int i=0; i<3; i++)
    {
        outer(1, 0) = normals(2, i);
        outer(2, 0) = -normals(1, i);
        outer(0, 1) = -normals(2, i);
        outer(2, 1) = normals(0, i);
        outer(0, 2) = normals(1, i);
        outer(1, 2) = -normals(0, i);

        normals.block(0, (i+2)*3, 3, 3).noalias() = outer * normals.block(0, 3, 3, 3);
    }

    // endpoint projection
    double proj_min_endpoints;
    double proj_max_endpoints;
    double min_dist = 1e10;
    
    for (int i=0; i<15; i++)
    {
        const Eigen::Vector3d& v = normals.col(i);
        const double norm = v.norm();
        if (norm > 1e-4)
        {
            proj_min_endpoints = v.dot(transform_.translation() - obb->transform_.translation());
            proj_max_endpoints = proj_min_endpoints;

            for (int j=0; j<6; j++)
            {
                const double d = std::abs(v.dot(normals.col(j)));
                proj_min_endpoints -= d * sizes(j);
                proj_max_endpoints += d * sizes(j);
            }

            // outside determination
            if (proj_min_endpoints * proj_max_endpoints >= 0)
                return 0.;

            proj_min_endpoints /= norm;
            proj_max_endpoints /= norm;
         
            if (min_dist > -proj_min_endpoints)
                min_dist = -proj_min_endpoints;
            if (min_dist > proj_max_endpoints)
                min_dist = proj_max_endpoints;
        }
    }

    return min_dist;
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
