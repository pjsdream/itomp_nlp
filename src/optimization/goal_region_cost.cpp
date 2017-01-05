#include <itomp_nlp/optimization/goal_region_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp_optimization
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

    for (int j=0; j<goal_planes_.size(); j++)
    {
        const GoalPlane& goal = goal_planes_[j];

        const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(goal.link_id);

        Eigen::Vector4d ee_translation;
        ee_translation.block(0, 0, 3, 1) = link_transform * goal.translation;
        ee_translation(3) = 1.;

        const Eigen::Vector4d& plane = goal.plane;

        double v = plane.dot(ee_translation);

        // ReLU-like objective function
        if (v < 0)
        cost += -v;
    }

    return cost * weight_;
}

void GoalRegionCost::addGoalRegionPlane(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector4d& plane)
{
    GoalPlane goal;
    goal.link_id = link_id;
    goal.translation = translation;
    goal.plane = plane;

    goal_planes_.push_back(goal);
}

double GoalRegionCost::f(double x)
{
    if (x < 0.01) return 0.;
    return x - 0.01;
}

}
