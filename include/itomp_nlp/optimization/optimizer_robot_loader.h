#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H


#include <itomp_nlp/optimization/optimizer_robot.h>


namespace itomp_optimization
{

class OptimizerRobotLoader
{
public:

    OptimizerRobotLoader();

    OptimizerRobot* loadRobot(itomp_robot::RobotModel* robot_model, itomp_robot::RobotState* robot_state, const std::vector<std::string>& active_joint_names);

private:

    void loadRobotRecursive(const itomp_robot::Link* link, const Eigen::Affine3d& transform, int parent_id);

    std::vector<std::string> active_joint_names_;
    itomp_robot::RobotState* robot_state_;

    std::vector<OptimizerRobot::Link> optimizer_links_;
    std::vector<OptimizerRobot::Joint> optimizer_joints_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H