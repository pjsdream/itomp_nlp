#ifndef ITOMP_ROBOT_JOINT_H
#define ITOMP_ROBOT_JOINT_H


#include <string>

#include <Eigen/Dense>


namespace itomp_robot
{

class Link;

class Joint
{
public:

    Joint();

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

    inline const Eigen::Affine3d& getOrigin()
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