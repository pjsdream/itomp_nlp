#ifndef ITOMP_SHAPE_OBB_H
#define ITOMP_SHAPE_OBB_H


#include <itomp_nlp/shape/shape.h>


namespace itomp
{

class AABB;
class Capsule2;

class OBB : public Shape
{
public:

    /// box [-x/2, x/2] x [-y/2, y/2] x [-z/2, z/2]
    OBB(double x, double y, double z);
    OBB(double x, double y, double z, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());

    OBB(const AABB& aabb);

    /// centered at (0, 0, 0)
    OBB(const Eigen::Vector3d& size);
    OBB(const Eigen::Vector3d& size, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());

    inline virtual Shape* clone() const
    {
        return new OBB(*this);
    }

    virtual double getPenetrationDepth(Shape* shape) const;
    double getPenetrationDepth(OBB* obb) const;
    double getPenetrationDepth(Capsule2* capsule) const;

    inline const Eigen::Vector3d& getSize() const
    {
        return size_;
    }

private:

    double distanceToPointNoTransform(const Eigen::Vector3d& p) const;

    // extents
    Eigen::Vector3d size_;
};

}


#endif // ITOMP_SHAPE_OBB_H