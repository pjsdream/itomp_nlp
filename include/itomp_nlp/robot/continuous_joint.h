#ifndef ITOMP_ROBOT_CONTINUOUS_JOINT_H
#define ITOMP_ROBOT_CONTINUOUS_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp
{

class ContinuousJoint : public Joint
{
public:

    ContinuousJoint();

    virtual double getDefaultPosition();
    virtual const ContinuousJoint* continuousJoint() const { return this; }

    virtual Eigen::Affine3d getTransform(double angle) const;

    inline const Eigen::Vector3d& getAxis() const
    {
        return axis_;
    }

    inline void setAxis(const Eigen::Vector3d& axis)
    {
        axis_ = axis;
    }

private:

    // axis
    Eigen::Vector3d axis_;
};

}


#endif // ITOMP_ROBOT_CONTINUOUS_JOINT_H