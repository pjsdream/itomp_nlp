#ifndef ITOMP_SHAPE_CAPSULE2_H
#define ITOMP_SHAPE_CAPSULE2_H


#include <itomp_nlp/shape/shape.h>


namespace itomp
{

class Capsule2 : public Shape
{
public:

    Capsule2();
    Capsule2(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rd);
    
    inline virtual Shape* clone() const
    {
        return new Capsule2(*this);
    }

    void setCapsule(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rq);

    inline const Eigen::Vector3d& getP() const
    {
        return p_[0];
    }

    inline const Eigen::Vector3d& getQ() const
    {
        return p_[1];
    }

    inline const double& getRp() const
    {
        return r_[0];
    }

    inline const double& getRq() const
    {
        return r_[1];
    }

private:

    Eigen::Vector3d p_[2];
    double r_[2];
};

}


#endif // ITOMP_SHAPE_CAPSULE2_H