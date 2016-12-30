#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_H


#include <itomp_nlp/optimization/optimizer_robot.h>

#include <Eigen/Dense>


namespace itomp_optimization
{

struct OptimizerOptions
{
    double trajectory_duration;
    double timestep;
};

class Optimizer
{
public:

    Optimizer();

    void setOptions(const OptimizerOptions& options);
    void setInitialRobotState(const Eigen::VectorXd& v);

private:
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_H