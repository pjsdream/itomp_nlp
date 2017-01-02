#include <itomp_nlp/robot/robot_state.h>

#include <itomp_nlp/robot/robot_model.h>
#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

RobotState::RobotState(const RobotModel* robot_model)
{
    const std::vector<std::string>& joint_names = robot_model->getJointNames();

    for (int i=0; i<joint_names.size(); i++)
    {
        Joint* joint = robot_model->getJoint( joint_names[i] );
        setPosition(joint_names[i], joint->getDefaultPosition());
    }
}

void RobotState::setPosition(const std::string& joint_name, double value)
{
    positions_[joint_name] = value;
}

double RobotState::getPosition(const std::string& joint_name)
{
    return positions_[joint_name];
}

}