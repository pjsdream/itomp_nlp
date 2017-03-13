#ifndef ITOMP_ROBOT_JOINT_H
#define ITOMP_ROBOT_JOINT_H


#include <string>

#include <Eigen/Dense>


namespace itomp
{

class Link;

class PrismaticJoint;
class ContinuousJoint;
class RevoluteJoint;
class FixedJoint;

class Joint
{
public:

    Joint();

    virtual double getDefaultPosition() = 0;

    virtual const PrismaticJoint* prismaticJoint() const { return 0; }
    virtual const ContinuousJoint* continuousJoint() const { return 0; }
    virtual const RevoluteJoint* revoluteJoint() const { return 0; }
    virtual const FixedJoint* fixedJoint() const { return 0; }

    virtual Eigen::Affine3d getTransform(double q) const = 0;

    inline const std::string& getJointName() const
    {
        return joint_name_;
    }

    inline void setJointName(const std::string& name)
    {
        joint_name_ = name;
    }

    inline Link* getChildLink() const
    {
        return child_link_;
    }

    inline const Eigen::Affine3d& getOrigin() const
    {
        return origin_;
    }

    void setParentLink(Link* link);
    void setChildLink(Link* link);

    void setOrigin(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

protected:

    std::string joint_name_;

    Link* parent_link_;
    Link* child_link_;

    Eigen::Affine3d origin_;
};

}


#endif // ITOMP_ROBOT_JOINT_H