#include <itomp_nlp/robot/continuous_joint.h>


namespace itomp_robot
{

ContinuousJoint::ContinuousJoint()
    : Joint()
{
}

double ContinuousJoint::getDefaultPosition()
{
    return 0.;
}

Eigen::Affine3d ContinuousJoint::getTransform(double angle) const
{
    return Eigen::Affine3d( Eigen::AngleAxisd(angle, axis_) );
}

}
