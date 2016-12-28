#include <itomp_nlp/robot/robot_model.h>


namespace itomp_robot
{

RobotModel::RobotModel()
{
}

void RobotModel::setRootLink(Link* link)
{
    root_link_ = link;
}

}
