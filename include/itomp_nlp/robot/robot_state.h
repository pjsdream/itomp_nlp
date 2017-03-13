#ifndef ITOMP_ROBOT_ROBOT_STATE_H
#define ITOMP_ROBOT_ROBOT_STATE_H


#include <string>
#include <map>


namespace itomp
{

class RobotModel;

class RobotState
{
public:

    RobotState(const RobotModel* robot_model);

    void setPosition(const std::string& joint_name, double value);

    double getPosition(const std::string& joint_name);

private:

    std::map<std::string, double> positions_;
};

}


#endif // ITOMP_ROBOT_ROBOT_STATE_H