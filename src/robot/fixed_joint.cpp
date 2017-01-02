#include <itomp_nlp/robot/fixed_joint.h>


namespace itomp_robot
{

FixedJoint::FixedJoint()
    : Joint()
{
}

double FixedJoint::getDefaultPosition()
{
    return 0.;
}

Eigen::Affine3d FixedJoint::getTransform(double) const
{
    return Eigen::Affine3d::Identity();
}

}
