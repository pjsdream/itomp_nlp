#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/optimizer.h>

#include <iostream>


namespace itomp_optimization
{

GoalCost::GoalCost(Optimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double GoalCost::cost()
{
    double cost = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
    {
        OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[i];

        for (int j=0; j<goal_positions_.size(); j++)
        {
            const Goal& goal = goal_positions_[j];

            const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal.link_id);
            const Eigen::Vector3d ee_translation = link_transform.translation() + goal.translation;

            cost += (ee_translation - goal.goal_position).squaredNorm();
        }
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

}