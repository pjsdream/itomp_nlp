#include <itomp_nlp/optimization/repulsive_cost.h>
#include <itomp_nlp/optimization/optimizer.h>

#include <iostream>


namespace itomp_optimization
{

RepulsiveCost::RepulsiveCost(Optimizer& optimizer, double weight)
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

    for (int j=0; j<repulsions_.size(); j++)
    {
        const Repulsion& repulsion = repulsions_[j];

        const Eigen::Affine3d& link_transform = robot->getLinkWorldTransform(repulsion.link_id);
        const Eigen::Vector3d ee_translation = link_transform * repulsion.translation;

        cost += f( (ee_translation - repulsion.repulsion_center).norm(), repulsion.distance );
    }

    return cost * weight_;
}

void RepulsiveCost::addRepulsion(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& repulsion_center, double distance)
{
    Repulsion repulsion;
    repulsion.link_id = link_id;
    repulsion.translation = translation;
    repulsion.repulsion_center = repulsion_center;
    repulsion.distance = distance;

    repulsions_.push_back(repulsion);
}

double RepulsiveCost::f(double x, double d)
{
    return x < d ? (d-x) : 0.;
}

}
