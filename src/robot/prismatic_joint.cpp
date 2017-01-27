#include <itomp_nlp/robot/prismatic_joint.h>


namespace itomp
{

PrismaticJoint::PrismaticJoint()
    : Joint()
{
}

double PrismaticJoint::getDefaultPosition()
{
    return 0.;
}

Eigen::Affine3d PrismaticJoint::getTransform(double t) const
{
    return Eigen::Affine3d( Eigen::Translation3d(t * axis_) );
}

}
