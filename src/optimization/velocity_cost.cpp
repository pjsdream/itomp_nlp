#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/optimizer.h>

#include <iostream>


namespace itomp_optimization
{

VelocityCost::VelocityCost(Optimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double VelocityCost::cost()
{
    double cost = 0.;

    const double timestep = (double)optimizer_.trajectory_duration_ / (optimizer_.interpolated_variables_.cols() - 1);

    // TODO: half computation time
    for (int i = 1; i < optimizer_.forward_kinematics_robots_.size() - 1; i++)
    {
        OptimizerRobot* backward_robot = optimizer_.forward_kinematics_robots_[i-1];
        OptimizerRobot* forward_robot = optimizer_.forward_kinematics_robots_[i+1];

        for (int j=0; j<velocities_.size(); j++)
        {
            const GoalVelocity& goal_velocity = velocities_[j];

            const Eigen::Affine3d& backward_link_transform = backward_robot->getLinkWorldTransform(goal_velocity.link_id);
            const Eigen::Vector3d backward_ee_translation = backward_link_transform.translation() + goal_velocity.translation;

            const Eigen::Affine3d& forward_link_transform = forward_robot->getLinkWorldTransform(goal_velocity.link_id);
            const Eigen::Vector3d forward_ee_translation = forward_link_transform.translation() + goal_velocity.translation;

            const Eigen::Vector3d velocity = (forward_ee_translation - backward_ee_translation) / (2. * timestep);
            cost += (velocity - goal_velocity.velocity).squaredNorm();
        }
    }

    return cost * weight_;
}

void VelocityCost::addGoalVelocity(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& velocity)
{
    GoalVelocity goal;
    goal.link_id = link_id;
    goal.translation = translation;
    goal.velocity = velocity;

    velocities_.push_back(goal);
}

}
