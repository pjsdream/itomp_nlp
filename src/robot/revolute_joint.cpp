#include <itomp_nlp/robot/revolute_joint.h>


namespace itomp_robot
{

RevoluteJoint::RevoluteJoint()
    : Joint()
{
}

double RevoluteJoint::getDefaultPosition()
{
    return 0.;
}

Eigen::Affine3d RevoluteJoint::getTransform(double angle) const
{
    return Eigen::Affine3d( Eigen::AngleAxisd(angle, axis_) );
}

}
