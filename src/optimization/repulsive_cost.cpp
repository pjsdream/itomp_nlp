#include <itomp_nlp/optimization/repulsive_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <iostream>


namespace itomp_optimization
{

RepulsiveCost::RepulsiveCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double RepulsiveCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double RepulsiveCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(repulsion_.link_id);
    const Eigen::Vector3d ee_translation = link_transform * repulsion_.translation;

    cost += f( (ee_translation - repulsion_.repulsion_center).norm(), repulsion_.distance );

    return cost * weight_;
}

void RepulsiveCost::setRepulsion(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& repulsion_center, double distance)
{
    Repulsion repulsion;
    repulsion.link_id = link_id;
    repulsion.translation = translation;
    repulsion.repulsion_center = repulsion_center;
    repulsion.distance = distance;

    repulsion_ = repulsion;
}

double RepulsiveCost::f(double x, double d)
{
    // squared norm
    return x < d ? (d-x) * (d-x) : 0.;
}

}
