#include <itomp_nlp/optimization/optimizer.h>


namespace itomp_optimization
{

Optimizer::Optimizer()
{
}

void Optimizer::setOptions(const OptimizerOptions& options)
{
}

void Optimizer::setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity)
{
    // TODO
}

void Optimizer::setRobot(OptimizerRobot* robot)
{
    robot_ = robot;
}

void Optimizer::initialize()
{
    // TODO
}

}
