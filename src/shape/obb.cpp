#include <itomp_nlp/shape/obb.h>

#include <itomp_nlp/shape/aabb.h>
#include <itomp_nlp/shape/capsule2.h>

#include <iostream>


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
    // find normal candidates
    Eigen::Matrix<double, 3, 15> normals;

    const Eigen::Matrix3d R1 = transform_.linear();
    const Eigen::Matrix3d R2 = obb->getTransform().linear();
    
    int normal_idx = 0;

    for (int i=0; i<3; i++)
        normals.col(normal_idx++) = R1.col(i);

    for (int i=0; i<3; i++)
        normals.col(normal_idx++) = R2.col(i);

    for (int i=0; i<3; i++)
    {
        const Eigen::Vector3d c1 = R1.col(i);
        for (int j=0; j<3; j++)
        {
            const Eigen::Vector3d c2 = R2.col(j);

            if (std::abs(c1.dot(c2)) >= 1 - 1e-6)
                normals.col(normal_idx++) = c1;
            else
                normals.col(normal_idx++) = c1.cross(c2).normalized();
        }
    }

    // endpoints
    EigenAlignedVector<Eigen::Vector3d> endpoints[2];
    endpoints[0] = getEndpoints();
    endpoints[1] = obb->getEndpoints();

    // endpoint projection
    const double sign[2] = {1, -1};
    double proj_min_endpoints[2][15];
    double proj_max_endpoints[2][15];
    double proj_min[15];
    double proj_max[15];

    for (int i=0; i<2; i++)
    {
        for (int j=0; j<15; j++)
        {
            for (int k=0; k<endpoints[i].size(); k++)
            {
                const double proj = sign[i] * endpoints[i][k].dot(normals.col(j));
                if (k==0 || proj_min_endpoints[i][j] > proj)
                    proj_min_endpoints[i][j] = proj;
                if (k==0 || proj_max_endpoints[i][j] < proj)
                    proj_max_endpoints[i][j] = proj;
            }
        }
    }

    for (int i=0; i<15; i++)
    {
        proj_min[i] = proj_min_endpoints[0][i] + proj_min_endpoints[1][i];
        proj_max[i] = proj_max_endpoints[0][i] + proj_max_endpoints[1][i];
    }

    // compute penetration depth
    double min_dist = 1e10;
    for (int i=0; i<15; i++)
    {
        // outside determination
        if (proj_min[i] * proj_max[i] >= 0)
            return 0.;
         
        if (min_dist > -proj_min[i])
            min_dist = -proj_min[i];
        if (min_dist > proj_max[i])
            min_dist = proj_max[i];
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
