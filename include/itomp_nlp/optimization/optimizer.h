#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_H


#include <itomp_nlp/optimization/optimizer_robot.h>

#include <Eigen/Dense>

#include <thread>
#include <atomic>


namespace itomp_optimization
{

struct OptimizerOptions
{
    double trajectory_duration;
    double timestep;
    int num_waypoints;
    int num_waypoint_interpolations;
};

class Optimizer
{
public:

    Optimizer();
    ~Optimizer();

    void setOptions(const OptimizerOptions& options);
    void setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity);

    void setRobot(OptimizerRobot* robot);

    void initialize();

    // thread
    void startOptimizationThread();
    void stopOptimizationThread();

private:

    // should be called in the created thread
    void threadEnter();
    void optimize();

    std::thread optimization_thread_;
    std::atomic_bool thread_stop_mutex_;
    bool thread_stop_requested_;

    void interpolate();
    void forwardKinematics();

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
