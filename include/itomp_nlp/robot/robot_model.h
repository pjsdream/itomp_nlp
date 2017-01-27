#ifndef ITOMP_ROBOT_ROBOT_MODEL_H
#define ITOMP_ROBOT_ROBOT_MODEL_H


#include <string>
#include <map>

#include <itomp_nlp/robot/link.h>


namespace itomp
{

class RobotModel
{
public:

    RobotModel();
    ~RobotModel();

    inline const std::string& getRobotName() const
    {
        return robot_name_;
    }

    inline void setRobotName(const std::string& name)
    {
        robot_name_ = name;
    }

    inline const Link* getRootLink() const
    {
        return root_link_;
    }

    inline const std::vector<std::string>& getJointNames() const
    {
        return joint_names_;
    }

    inline Joint* getJoint(const std::string& joint_name) const
    {
        std::map<std::string, Joint*>::const_iterator it = map_joint_.find(joint_name);

        // TODO: check existance of joint

        return it->second;
    }

    void setRootLink(Link* link);

private:

    void initializeJointLinkNames(Link* link);

    std::string robot_name_;

    Link* root_link_;

    // joint/link name cache
    std::vector<std::string> link_names_;
    std::vector<std::string> joint_names_;
    std::map<std::string, Link*> map_link_;
    std::map<std::string, Joint*> map_joint_;
};

}


#endif // ITOMP_ROBOT_ROBOT_MODEL_H