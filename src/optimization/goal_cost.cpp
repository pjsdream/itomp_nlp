#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <iostream>


namespace itomp_optimization
{

GoalCost::GoalCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double GoalCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double GoalCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    for (int j=0; j<goal_positions_.size(); j++)
    {
        const Goal& goal = goal_positions_[j];

        const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal.link_id);
        const Eigen::Vector3d ee_translation = link_transform * goal.translation;

        // ReLU-like objective function
        cost += (ee_translation - goal.goal_position).squaredNorm();
    }

    return cost * weight_;
}

void GoalCost::addGoalPosition(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position)
{
    Goal goal;
    goal.link_id = link_id;
    goal.translation = translation;
    goal.goal_position = goal_position;

    goal_positions_.push_back(goal);
}

double GoalCost::f(double x)
{
    if (x < 0.01) return 0.;
    return x - 0.01;
}

}
