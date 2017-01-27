#ifndef ITOMP_SHAPE_SHAPE_H
#define ITOMP_SHAPE_SHAPE_H


#include <Eigen/Dense>


namespace itomp
{

class Shape
{
public:

    Shape();
    Shape(const Eigen::Affine3d& transform);
    ~Shape();

    inline virtual Shape* clone() const
    {
        return new Shape(*this);
    }

    inline void setTransform(const Eigen::Affine3d& transform)
    {
        transform_ = transform;
    }

    inline const Eigen::Affine3d& getTransform() const
    {
        return transform_;
    }

    void applyTransform(const Eigen::Affine3d& transform);

protected:

    Eigen::Affine3d transform_;
};

}


#endif // ITOMP_SHAPE_SHAPE_H