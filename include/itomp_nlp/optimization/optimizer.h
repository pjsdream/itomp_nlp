#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_H


#include <itomp_nlp/optimization/optimizer_robot.h>
#include <itomp_nlp/optimization/cost.h>

#include <Eigen/Dense>

#include <thread>
#include <atomic>
#include <mutex>


namespace itomp_optimization
{

struct OptimizerOptions
{
    double trajectory_duration;
    double timestep;
    int num_waypoints;
    int num_waypoint_interpolations;
};

/**
 *  function call order
 *  1. setOptions, setRobot
 *  2. prepare
 *  3. setInitialRobotState, setGoalPosition
 */
class Optimizer
{
public:

    friend class SmoothnessCost;
    friend class CollisionCost;
    friend class GoalCost;

    enum CostFunctionType
    {
        SMOOTHNESS_COST = 0,
        COLLISION_COST,
        GOAL_COST,
        NUM_COST_FUNCTIONS
    };

public:

    Optimizer();
    ~Optimizer();

    void setOptions(const OptimizerOptions& options);
    void setRobot(OptimizerRobot* robot);

    void prepare();

    void setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity);

    // goal position
    void setGoalPosition(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position);

    // thread
    void startOptimizationThread();
    void stopOptimizationThread();

    void getBestTrajectory();

private:

    // should be called in the created thread
    void threadEnter();
    void optimize();

    std::thread optimization_thread_;
    std::atomic_bool thread_stop_mutex_;
    bool thread_stop_requested_;

    // precomputation
    void optimizationPrecomputation();
    void interpolate();
    void forwardKinematics();

    // objective function and gradient computation
    double cost();
    void computeGradient();
    Eigen::MatrixXd gradient_;

    // cost functions
    void initializeCostFunctions();
    std::vector<Cost*> cost_functions_;
    
    // best result trajectory
    void storeBestWaypointVariables();
    std::mutex mutex_best_waypoint_variables_;
    Eigen::MatrixXd best_waypoint_variables_;

    OptimizerRobot* robot_;

    double trajectory_duration_;
    double timestep_;
    int num_waypoints_;
    int num_waypoint_interpolations_;

    // forward kinematics robot
    std::vector<OptimizerRobot*> forward_kinematics_robots_;

    // waypoint variables
    // [q0 q0' q1 q1' q2 q2' ... ]
    int dof_;
    Eigen::MatrixXd waypoint_variables_;

    // interpolated variables
    // [q0 q0' q1 q1' q2 q2' ... ]
    Eigen::MatrixXd interpolation_coefficients_;
    Eigen::MatrixXd interpolated_variables_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_H
