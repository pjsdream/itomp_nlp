#ifndef ITOMP_ROBOT_FIXED_JOINT_H
#define ITOMP_ROBOT_FIXED_JOINT_H


#include <itomp_nlp/robot/joint.h>


namespace itomp
{

class FixedJoint : public Joint
{
public:

    FixedJoint();

    virtual double getDefaultPosition();
    virtual const FixedJoint* fixedJoint() const { return this; }

    virtual Eigen::Affine3d getTransform(double) const;

private:
};

}


#endif // ITOMP_ROBOT_FIXED_JOINT_H