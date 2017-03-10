#include <itomp_nlp/optimization/goal_upvector_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <iostream>


namespace itomp
{

GoalUpvectorCost::GoalUpvectorCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double GoalUpvectorCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double GoalUpvectorCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal_.link_id);
    const Eigen::Vector3d ee_upvector(link_transform.linear().block(0, 2, 3, 1));

    double cosangle = ee_upvector.dot(goal_.goal_upvector);
    cost = (1. - cosangle) * (1. - cosangle);
     
    return cost * weight_;
}

void GoalUpvectorCost::setGoalUpvector(int link_id, const Eigen::Vector3d& goal_upvector)
{
    goal_.link_id = link_id;
    goal_.goal_upvector = goal_upvector;
}

}
