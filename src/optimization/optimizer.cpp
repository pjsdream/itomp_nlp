#include <itomp_nlp/optimization/optimizer.h>

#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/goal_region_cost.h>

#include <functional>

#include <iostream>


namespace itomp_optimization
{

Optimizer::Optimizer()
{
}

Optimizer::~Optimizer()
{
    // delete generated/cloned objects
    delete robot_;

    for (int i=0; i<forward_kinematics_robots_.size(); i++)
        delete forward_kinematics_robots_[i];

    for (int i=0; i<cost_functions_.size(); i++)
        delete cost_functions_[i];
}

void Optimizer::setOptions(const OptimizerOptions& options)
{
    trajectory_duration_ = options.trajectory_duration;
    timestep_ = options.timestep;
    num_waypoints_ = options.num_waypoints;
    num_waypoint_interpolations_ = options.num_waypoint_interpolations;
}

void Optimizer::setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity)
{
    waypoint_variables_.col(0) = position;
    waypoint_variables_.col(1) = velocity;
}

void Optimizer::setRobot(OptimizerRobot* robot)
{
    robot_ = robot->clone();
}

void Optimizer::prepare()
{
    // forward kinematics robot model
    const int num_interpolations = (num_waypoint_interpolations_ + 1) * num_waypoints_ + 1;
    forward_kinematics_robots_.resize(num_interpolations);

    for (int i=0; i<num_interpolations; i++)
        forward_kinematics_robots_[i] = robot_->clone();

    // waypoint variables
    dof_ = robot_->getDoF();
    waypoint_variables_.resize(dof_, (num_waypoints_ + 1) * 2);
    waypoint_variables_.setZero();

    // interpolated variables
    interpolated_variables_.resize(dof_, num_interpolations * 2);

    // hermite interpolation coefficients
    const double T = trajectory_duration_ / num_waypoints_;
    interpolation_coefficients_.resize(4, (num_waypoint_interpolations_ + 2) * 2);
    for (int i=0; i <= num_waypoint_interpolations_ + 1; i++)
    {
        const double t = (double)i / (num_waypoint_interpolations_ + 1);
        const double t2 = t * t;
        const double t3 = t2 * t;

        const double h0 = 2. * t3 - 3. * t2 + 1.;
        const double h1 = t3 - 2. * t2 + t;
        const double h2 = -2. * t3 + 3. * t2;
        const double h3 = t3 - t2;

        const double h0p = 6. * t2 - 6. * t;
        const double h1p = 3. * t2 - 4. * t + 1.;
        const double h2p = -6. * t2 + 6. * t;
        const double h3p = 3. * t2 - 2. * t;

        interpolation_coefficients_(0, i*2  ) = h0;
        interpolation_coefficients_(1, i*2  ) = h1 * T;
        interpolation_coefficients_(2, i*2  ) = h2;
        interpolation_coefficients_(3, i*2  ) = h3 * T;
        interpolation_coefficients_(0, i*2+1) = h0p;
        interpolation_coefficients_(1, i*2+1) = h1p * T;
        interpolation_coefficients_(2, i*2+1) = h2p;
        interpolation_coefficients_(3, i*2+1) = h3p * T;
    }

    // gradient resize
    gradient_.resize(dof_, num_waypoints_ * 2);

    // cost function initialization
    initializeCostFunctions();
}

void Optimizer::initializeCostFunctions()
{
    cost_functions_.resize(NUM_COST_FUNCTIONS);

    // smoothness cost
    SmoothnessCost* smoothness_cost = new SmoothnessCost(*this, 1);
    cost_functions_[SMOOTHNESS_COST] = smoothness_cost;

    // collision cost
    CollisionCost* collision_cost = new CollisionCost(*this, 1);
    cost_functions_[COLLISION_COST] = collision_cost;

    // goal cost
    GoalCost* goal_cost = new GoalCost(*this, 1);
    cost_functions_[GOAL_COST] = goal_cost;

    // velocity cost
    VelocityCost* velocity_cost = new VelocityCost(*this, 1);
    cost_functions_[VELOCITY_COST] = velocity_cost;

    // goal region cost
    GoalRegionCost* goal_region_cost = new GoalRegionCost(*this, 1);
    cost_functions_[GOAL_REGION_COST] = goal_region_cost;
}

void Optimizer::setGoalPosition(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position)
{
    GoalCost* goal_cost = dynamic_cast<GoalCost*>(cost_functions_[GOAL_COST]);
    goal_cost->addGoalPosition(link_id, translate, goal_position);
}

void Optimizer::setGoalVelocity(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector3d& goal_position, const Eigen::Vector3d& velocity)
{
    VelocityCost* velocity_cost = dynamic_cast<VelocityCost*>(cost_functions_[VELOCITY_COST]);
    velocity_cost->addGoalVelocity(link_id, translate, goal_position, velocity);
}

void Optimizer::addGoalRegionPlane(int link_id, const Eigen::Vector3d& translate, const Eigen::Vector4d& plane)
{
    GoalRegionCost* goal_region_cost = dynamic_cast<GoalRegionCost*>(cost_functions_[GOAL_REGION_COST]);
    goal_region_cost->addGoalRegionPlane(link_id, translate, plane);
}

void Optimizer::startOptimizationThread()
{
    thread_stop_requested_ = false;
    move_forward_requests_ = 0;

    optimization_thread_ = std::thread( std::bind(&Optimizer::threadEnter, this) );
}

void Optimizer::stopOptimizationThread()
{
    thread_stop_requested_ = true;

    optimization_thread_.join();
}

Eigen::MatrixXd Optimizer::getBestTrajectory()
{
    mutex_best_waypoint_variables_.lock();
    best_waypoint_variables_ = best_waypoint_variables_pass_;
    best_interpolated_variables_ = best_interpolated_variables_pass_;
    mutex_best_waypoint_variables_.unlock();

    return best_interpolated_variables_;
}

void Optimizer::moveForwardOneTimestep()
{
    move_forward_requests_++;
}

void Optimizer::threadEnter()
{
    optimize();
}

void Optimizer::updateWhileOptimizing()
{
    while (move_forward_requests_)
    {
        moveForwardOneTimestepInternal();
        move_forward_requests_--;
    }
}

void Optimizer::moveForwardOneTimestepInternal()
{
    int num_shift = (timestep_ + 1e-6) / ((double)trajectory_duration_ / num_waypoints_);
    waypoint_variables_.block(0, 0, dof_, waypoint_variables_.cols() - num_shift * 2) = waypoint_variables_.block(0, 2 * num_shift, dof_, waypoint_variables_.cols() - num_shift * 2); 

    for (int i = waypoint_variables_.cols() - num_shift * 2; i < waypoint_variables_.cols(); i += 2)
    {
        waypoint_variables_.col(i    ) = waypoint_variables_.col(i - 2);
        waypoint_variables_.col(i + 1).setZero();
    }
}

void Optimizer::optimize()
{
    int iterations = 0;

    const double alpha = 0.01;

    while (!thread_stop_requested_)
    {
        updateWhileOptimizing();
        
        optimizationPrecomputation();

        // cost function
        const double f = cost();

        // DEBUG: print cost functions
        for (int i=0; i<NUM_COST_FUNCTIONS; i++)
        {
            const double c = cost_functions_[i]->cost();
            printf("%.9lf ", c);
        }
        printf("\n");

        /*
        // DEBUG: print iteration and costs (slow)
        printf("iteration %5d: %lf\n", iterations, f);

        // DEBUG: endeffector (link 7) position print
        Eigen::Vector3d e = (*forward_kinematics_robots_.rbegin())->getLinkWorldTransform(7) * Eigen::Vector3d(0.1, 0, 0);
        printf("link 7 position: %lf %lf %lf\n", e(0), e(1), e(2));
        */

        // simple gradient descent update
        computeGradient();

        // update current point
        waypoint_variables_.block(0, 2, dof_, num_waypoints_ * 2) -= alpha * gradient_;

        // update as the best trajectory
        storeBestWaypointVariables();

        iterations++;
    }
}

void Optimizer::storeBestWaypointVariables()
{
    mutex_best_waypoint_variables_.lock();
    best_waypoint_variables_pass_ = waypoint_variables_;
    best_interpolated_variables_pass_ = interpolated_variables_;
    mutex_best_waypoint_variables_.unlock();
}

double Optimizer::cost()
{
    double cost = 0.;

    for (int i=0; i<cost_functions_.size(); i++)
        cost += cost_functions_[i]->cost();

    return cost;
}

void Optimizer::computeGradient()
{
    gradient_.setZero();
    
    // TODO: optimize by using locality of trajectory, or using analytic gradient

    // central difference
    const double delta = 0.0001;
    for (int i=2; i < waypoint_variables_.cols(); i++)
    {
        for (int j=0; j < waypoint_variables_.rows(); j++)
        {
            const double original_value = waypoint_variables_(j, i);

            waypoint_variables_(j, i) += delta;
            
            optimizationPrecomputation();
            const double forward_cost = cost();

            waypoint_variables_(j, i) -= 2. * delta;

            optimizationPrecomputation();
            const double backward_cost = cost();

            waypoint_variables_(j, i) = original_value;

            gradient_(j, i-2) = (forward_cost - backward_cost) / (2. * delta);
        }
    }

    // restore interpolation variables modified by central difference method
    optimizationPrecomputation();
}

void Optimizer::optimizationPrecomputation()
{
    interpolate();
    forwardKinematics();
}

void Optimizer::interpolate()
{
    for (int i=0; i<num_waypoints_; i++)
        interpolated_variables_.block(0, 2 * i * (num_waypoint_interpolations_ + 1), dof_, (num_waypoint_interpolations_ + 2) * 2) = waypoint_variables_.block(0, i * 2, dof_, 4) * interpolation_coefficients_;
}

void Optimizer::forwardKinematics()
{
    for (int i=0; i<forward_kinematics_robots_.size(); i++)
    {
        forward_kinematics_robots_[i]->setPositions (interpolated_variables_.col(2*i  ));
        forward_kinematics_robots_[i]->setVelocities(interpolated_variables_.col(2*i+1));

        forward_kinematics_robots_[i]->forwardKinematics();
    }
}

}
