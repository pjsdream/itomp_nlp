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

void Optimizer::setZeroCost(int id)
{
    Cost* cost = new Cost(optimization_thread_, 0.0);
    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setSmoothnessCost(int id, double weight)
{
    SmoothnessCost* cost = new SmoothnessCost(optimization_thread_, weight);
    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setGoalCost(int id, double weight, int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position)
{
    GoalCost* cost = new GoalCost(optimization_thread_, weight);
    cost->setGoalPosition(link_id, translation, goal_position);

    optimization_thread_.pushCostFunctionRequest(id, cost);
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
