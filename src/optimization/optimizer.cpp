#include <itomp_nlp/optimization/optimizer.h>

#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/goal_orientation_cost.h>
#include <itomp_nlp/optimization/goal_upvector_cost.h>
#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/goal_region_cost.h>
#include <itomp_nlp/optimization/repulsive_cost.h>

#include <functional>

#include <iostream>


namespace itomp
{

Optimizer::Optimizer()
{
    scene_ = new Scene();
}

Optimizer::~Optimizer()
{
    delete scene_;
}

void Optimizer::addStaticObstacle(StaticObstacle* obstacle)
{
    scene_->addStaticObstacle(obstacle);
}

void Optimizer::addDynamicObstacle(DynamicObstacle* obstacle)
{
    scene_->addDynamicObstacle(obstacle);
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

void Optimizer::setCollisionCost(int id, double weight)
{
    CollisionCost* cost = new CollisionCost(optimization_thread_, scene_, weight);
    cost->updateSceneObstacles();
    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setGoalCost(int id, double weight, int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position)
{
    GoalCost* cost = new GoalCost(optimization_thread_, weight);
    cost->setGoalPosition(link_id, translation, goal_position);

    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setGoalOrientationCost(int id, double weight, int link_id, const Eigen::Quaterniond& quaternion)
{
    GoalOrientationCost* cost = new GoalOrientationCost(optimization_thread_, weight);
    cost->setGoalOrientation(link_id, quaternion);

    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setGoalUpvectorCost(int id, double weight, int link_id, const Eigen::Vector3d& upvector)
{
    GoalUpvectorCost* cost = new GoalUpvectorCost(optimization_thread_, weight);
    cost->setGoalUpvector(link_id, upvector);

    optimization_thread_.pushCostFunctionRequest(id, cost);
}

void Optimizer::setVelocityCost(int id, double weight, int link_id, const Eigen::Vector3d& velocity)
{
    VelocityCost* cost = new VelocityCost(optimization_thread_, weight);
    cost->setGoalVelocity(link_id, velocity);

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

void Optimizer::updateScene()
{
    optimization_thread_.updateScene();
}

double Optimizer::getBestTrajectoryCost()
{
    return optimization_thread_.getBestTrajectoryCost();
}

void Optimizer::changeGoalCost()
{
    optimization_thread_.changeGoalCost();
}

}
