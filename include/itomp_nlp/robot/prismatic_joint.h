#ifndef ITOMP_ROBOT_PRISMATIC_JOINT_H
#define ITOMP_ROBOT_PRISMATIC_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

class PrismaticJoint : public Joint
{
public:

    PrismaticJoint();

    virtual double getDefaultPosition();
    virtual const PrismaticJoint* prismaticJoint() const { return this; }

    virtual Eigen::Affine3d getTransform(double t) const;

    inline const Eigen::Vector3d& getAxis() const
    {
        return axis_;
    }

    inline void setAxis(const Eigen::Vector3d& axis)
    {
        axis_ = axis;
    }

    inline void setLimit(double lower, double upper)
    {
        lower_ = lower;
        upper_ = upper;
    }

    inline double getPositionLowerLimit() const
    {
        return lower_;
    }

    inline double getPositionUpperLimit() const
    {
        return upper_;
    }

private:

    // axis
    Eigen::Vector3d axis_;

    // limits
    double lower_;
    double upper_;
};

}


#endif // ITOMP_ROBOT_PRISMATIC_JOINT_H