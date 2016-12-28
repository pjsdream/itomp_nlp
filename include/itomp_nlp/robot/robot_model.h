#ifndef ITOMP_ROBOT_ROBOT_MODEL_H
#define ITOMP_ROBOT_ROBOT_MODEL_H


#include <string>

#include <itomp_nlp/robot/link.h>


namespace itomp_robot
{

class RobotModel
{
public:

    RobotModel();

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

    void setRootLink(Link* link);

private:

    std::string robot_name_;

    Link* root_link_;
};

}


#endif // ITOMP_ROBOT_ROBOT_MODEL_H