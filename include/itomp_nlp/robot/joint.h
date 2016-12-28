#ifndef ITOMP_ROBOT_JOINT_H
#define ITOMP_ROBOT_JOINT_H


#include <string>


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

    void setParentLink(Link* link);
    void setChildLink(Link* link);

    inline Link* getChildLink() const
    {
        return child_link_;
    }

private:

    std::string joint_name_;

    Link* parent_link_;
    Link* child_link_;
};

}


#endif // ITOMP_ROBOT_JOINT_H