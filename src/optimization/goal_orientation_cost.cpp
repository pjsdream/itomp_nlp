#include <itomp_nlp/optimization/goal_orientation_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <iostream>


namespace itomp
{

GoalOrientationCost::GoalOrientationCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double GoalOrientationCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double GoalOrientationCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal_.link_id);
    const Eigen::Quaterniond ee_orientation(link_transform.linear());

    double angle = ee_orientation.angularDistance(goal_.goal_orientation);
    cost = angle * angle;
     
    return cost * weight_;
}

void GoalOrientationCost::setGoalOrientation(int link_id, const Eigen::Quaterniond& goal_orientation)
{
    goal_.link_id = link_id;
    goal_.goal_orientation = goal_orientation;
}

}
