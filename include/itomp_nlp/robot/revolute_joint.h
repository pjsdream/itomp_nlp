#ifndef ITOMP_ROBOT_REVOLUTE_JOINT_H
#define ITOMP_ROBOT_REVOLUTE_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp
{

class RevoluteJoint : public Joint
{
public:

    RevoluteJoint();

    virtual double getDefaultPosition();
    virtual const RevoluteJoint* revoluteJoint() const { return this; }

    virtual Eigen::Affine3d getTransform(double angle) const;

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


#endif // ITOMP_ROBOT_REVOLUTE_JOINT_H