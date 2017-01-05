#include <itomp_nlp/optimization/optimizer.h>

#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/goal_region_cost.h>
#include <itomp_nlp/optimization/repulsive_cost.h>

#include <functional>

#include <iostream>


namespace itomp_optimization
{

Optimizer::Optimizer()
{
}

Optimizer::~Optimizer()
{
}

void Optimizer::setOptions(const OptimizerOptions& options)
{
    optimization_thread_.setTrajectoryDuration(options.trajectory_duration);
    optimization_thread_.setTimestep(options.timestep);
    optimization_thread_.setNumWaypoints(options.num_waypoints);
    optimization_thread_.setNumWaypointInterpolations(options.num_waypoint_interpolations);
}

void Optimizer::setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity)
{
    optimization_thread_.setInitialRobotState(position, velocity);
}

void Optimizer::setRobot(OptimizerRobot* robot)
{
    optimization_thread_.setRobot(robot);
}

void Optimizer::prepare()
{
    optimization_thread_.prepare();
}

void Optimizer::setGoalPosition(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position)
{
    optimization_thread_.setGoalPosition(link_id, translate, goal_position);
}

void Optimizer::setGoalVelocity(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position, const Eigen::Vector3d& velocity)
{
    optimization_thread_.setGoalVelocity(link_id, translate, goal_position, velocity);
}

void Optimizer::addGoalRegionPlane(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector4d& plane)
{
    optimization_thread_.addGoalRegionPlane(link_id, translate, plane);
}

void Optimizer::addRepulsion(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& repulsion_center, double distance)
{
    optimization_thread_.addRepulsion(link_id, translate, repulsion_center, distance);
}

void Optimizer::startOptimizationThread()
{
    optimization_thread_.start();
}

void Optimizer::stopOptimizationThread()
{
    optimization_thread_.stop();
}

Eigen::MatrixXd Optimizer::getBestTrajectory()
{
    return optimization_thread_.getBestTrajectory();
}

void Optimizer::moveForwardOneTimestep()
{
    optimization_thread_.moveForwardOneTimestep();
}

}
