#ifndef ITOMP_ROBOT_REVOLUTE_JOINT_H
#define ITOMP_ROBOT_REVOLUTE_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

class RevoluteJoint : public Joint
{
public:

    RevoluteJoint();

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

private:

    // axis
    Eigen::Vector3d axis_;

    // limits
    double lower_;
    double upper_;
};

}


#endif // ITOMP_ROBOT_REVOLUTE_JOINT_H