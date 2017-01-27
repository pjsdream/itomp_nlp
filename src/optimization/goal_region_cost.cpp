#include <itomp_nlp/optimization/goal_region_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp
{

GoalRegionCost::GoalRegionCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double GoalRegionCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double GoalRegionCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal_plane_.link_id);

    Eigen::Vector4d ee_translation;
    ee_translation.block(0, 0, 3, 1) = link_transform * goal_plane_.translation;
    ee_translation(3) = 1.;

    const Eigen::Vector4d& plane = goal_plane_.plane;

    double v = plane.dot(ee_translation);

    // ReLU-like objective function
    if (v < 0)
    cost += v*v;

    return cost * weight_;
}

void GoalRegionCost::setGoalPlane(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector4d& plane)
{
    GoalPlane goal;
    goal.link_id = link_id;
    goal.translation = translation;
    goal.plane = plane;

    goal_plane_ = goal;
}

}
