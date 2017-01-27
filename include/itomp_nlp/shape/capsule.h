#ifndef ITOMP_SHAPE_CAPSULE_H
#define ITOMP_SHAPE_CAPSULE_H


#include <itomp_nlp/shape/shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class Capsule : public Shape
{
public:

    Capsule();
    Capsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d);
    
    inline virtual Shape* clone() const
    {
        return new Capsule(*this);
    }

    void setCapsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d);

private:

    Eigen::Vector3d p_[2];
    double d_;
};

}


#endif // ITOMP_SHAPE_AABB_H