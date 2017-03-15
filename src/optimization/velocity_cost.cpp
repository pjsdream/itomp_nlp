#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <iostream>


namespace itomp
{

VelocityCost::VelocityCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double VelocityCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double VelocityCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    const Eigen::Vector3d ee_velocity = robot->getWorldVelocity(goal_.link_id, Eigen::Vector3d::Zero());

    cost += (ee_velocity - goal_.velocity).squaredNorm();
     
    return cost * weight_;
}

void VelocityCost::setGoalVelocity(int link_id, const Eigen::Vector3d& velocity)
{
    goal_.link_id = link_id;
    goal_.velocity = velocity;
}

double VelocityCost::f(double x)
{
    return x < 0.1 ? 0. : x - 0.1;
}

}
