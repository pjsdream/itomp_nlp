#ifndef ITOMP_SHAPE_AABB_H
#define ITOMP_SHAPE_AABB_H


#include <itomp_nlp/shape/shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class AABB : public Shape
{
public:

    AABB();
    AABB(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper);
    
    inline virtual Shape* clone() const
    {
        return new AABB(*this);
    }

    inline const Eigen::Vector3d& getLower() const
    {
        return lower_;
    }

    inline const Eigen::Vector3d& getUpper() const
    {
        return upper_;
    }

    AABB merge(const AABB& aabb);

    void translate(const Eigen::Vector3d& t);

private:

    Eigen::Vector3d lower_;
    Eigen::Vector3d upper_;
};

}


#endif // ITOMP_SHAPE_AABB_H