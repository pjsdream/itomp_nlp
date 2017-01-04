#include <itomp_nlp/optimization/optimizer.h>

#include <functional>


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

void Optimizer::initialize()
{
    // forward kinematics robot model
    const int num_interpolations = (num_waypoint_interpolations_ + 1) * num_waypoints_ + 1;
    forward_kinematics_robots_.resize(num_interpolations);

    for (int i=0; i<num_interpolations; i++)
        forward_kinematics_robots_[i] = robot_->clone();

    // waypoint variables
    dof_ = robot_->getDoF();
    waypoint_variables_.resize(dof_, num_waypoints_ * 2);
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
}

void Optimizer::startOptimizationThread()
{
    thread_stop_requested_ = false;
    optimization_thread_ = std::thread( std::bind(&Optimizer::threadEnter, this) );
}

void Optimizer::stopOptimizationThread()
{
    thread_stop_requested_ = true;
    optimization_thread_.join();
}

void Optimizer::threadEnter()
{
    optimize();
}

void Optimizer::optimize()
{
    while (!thread_stop_requested_)
    {
    }
}

void Optimizer::interpolate()
{
    for (int i=0; i<num_waypoints_; i++)
        interpolated_variables_.block(0, 2 * i * (num_waypoint_interpolations_ + 1), dof_, (num_waypoint_interpolations_ + 2) * 2) = waypoint_variables_.block(0, i * 2, dof_, 4) * interpolation_coefficients_;

    forwardKinematics();
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
