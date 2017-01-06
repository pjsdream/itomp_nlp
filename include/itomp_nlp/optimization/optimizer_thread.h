#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_THREAD_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_THREAD_H


#include <itomp_nlp/optimization/optimizer_robot.h>
#include <itomp_nlp/optimization/cost.h>

#include <Eigen/Dense>

#include <thread>
#include <atomic>
#include <mutex>

#include <dlib/optimization.h>


namespace itomp_optimization
{

class OptimizerThread
{
public:

    friend class SmoothnessCost;
    friend class CollisionCost;
    friend class GoalCost;
    friend class VelocityCost;
    friend class GoalRegionCost;
    friend class RepulsiveCost;

    enum CostFunctionType
    {
        SMOOTHNESS_COST = 0,
        COLLISION_COST,
        GOAL_COST,
        VELOCITY_COST,
        GOAL_REGION_COST,
        REPULSIVE_COST,
        NUM_COST_FUNCTIONS
    };

private:

    typedef dlib::matrix<double,0,1> column_vector;

public:

    OptimizerThread();
    ~OptimizerThread();

    inline int getNumInterpolatedVariables() const
    {
        return interpolated_variables_.cols() / 2;
    }

    inline void setTrajectoryDuration(double duration)
    {
        trajectory_duration_ = duration;
    }

    inline void setTimestep(double timestep)
    {
        timestep_ = timestep;
    }

    inline void setNumWaypoints(int num_waypoints)
    {
        num_waypoints_ = num_waypoints;
    }

    inline void setNumWaypointInterpolations(int num_waypoint_interpolations)
    {
        num_waypoint_interpolations_ = num_waypoint_interpolations;
    }

    void setRobot(OptimizerRobot* robot);

    void prepare();

    void setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity);

    // goal position
    void setGoalPosition(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position);

    // goal velocity
    void setGoalVelocity(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position, const Eigen::Vector3d& velocity);

    // goal plane
    void addGoalRegionPlane(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector4d& plane);

    // repulsion
    void addRepulsion(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& repulsion_center, double distance);

    // TODO: output format. Currently, returns interpolated joint positions and velocities
    Eigen::MatrixXd getBestTrajectory();

    void start();
    void stop();

    void moveForwardOneTimestep();

private:

    // should be called in the created thread
    void threadEnter();
    void optimizeGradientDescent();
    
    // dlib optimization
    template <
        typename search_strategy_type,
        typename funct, 
        typename funct_der, 
        typename T
        >
    void optimizeDlib(
        search_strategy_type search_strategy,
        const funct& f, 
        const funct_der& der, 
        T& x, 
        double min_f
        );
    column_vector dlib_waypoint_variables_;
    column_vector dlib_gradient_;
    double dlibCost(const column_vector& x);
    column_vector dlibDerivative(const column_vector& x);

    // thread
    std::thread thread_;
    std::atomic_bool thread_stop_mutex_;
    bool thread_stop_requested_;

    // precomputation
    void optimizationPrecomputation();
    void interpolate();
    void forwardKinematics();

    // objective function and gradient computation
    double cost();
    double cost(int interpolation_idx);
    void computeGradientDirect();
    void computeGradientChainRule();
    Eigen::MatrixXd gradient_;

    // cost functions
    void initializeCostFunctions();
    std::vector<Cost*> cost_functions_;
    
    // best result trajectory
    void storeBestWaypointVariables();
    std::mutex mutex_best_waypoint_variables_;
    Eigen::MatrixXd best_waypoint_variables_pass_;
    Eigen::MatrixXd best_waypoint_variables_;
    Eigen::MatrixXd best_interpolated_variables_pass_;
    Eigen::MatrixXd best_interpolated_variables_;

    // update while optimizing
    void updateWhileOptimizing();
    void moveForwardOneTimestepInternal();
    std::atomic_int move_forward_requests_;
    bool is_solution_updated_;

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
    Eigen::MatrixXd interpolation_coefficients_;  // 4 rows
    Eigen::MatrixXd interpolated_variables_;      // n rows
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_H
