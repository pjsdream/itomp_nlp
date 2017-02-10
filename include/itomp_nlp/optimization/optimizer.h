#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_H


#include <itomp_nlp/optimization/optimizer_robot.h>
#include <itomp_nlp/optimization/optimizer_thread.h>

#include <itomp_nlp/optimization/scene.h>

#include <Eigen/Dense>

#include <thread>
#include <atomic>
#include <mutex>


namespace itomp
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

public:

    Optimizer();
    ~Optimizer();

    // initialization
    void setOptions(const OptimizerOptions& options);
    void setRobot(RobotModel* robot);
    void prepare();

    // scene
    void addStaticObstacle(StaticObstacle* obstacle);
    void addDynamicObstacle(DynamicObstacle* obstacle);
    
    inline Scene* getScene() const
    {
        return scene_;
    }

    // initial robot state. updates the robot state for next timestep
    void setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity);

    // cost functions
    void setZeroCost(int id);
    void setSmoothnessCost(int id, double weight);
    void setCollisionCost(int id, double weight);
    void setGoalCost(int id, double weight, int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position);

    // thread
    void startOptimizationThread();
    void stopOptimizationThread();

    // TODO: output format. Currently, returns interpolated joint positions and velocities
    Eigen::MatrixXd getBestTrajectory();

    void moveForwardOneTimestep();

private:

    OptimizerThread optimization_thread_;

    Scene* scene_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_H
