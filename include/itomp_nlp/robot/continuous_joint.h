#ifndef ITOMP_ROBOT_CONTINUOUS_JOINT_H
#define ITOMP_ROBOT_CONTINUOUS_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

class ContinuousJoint : public Joint
{
public:

    ContinuousJoint();

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