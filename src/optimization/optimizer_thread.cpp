#include <itomp_nlp/optimization/optimizer_thread.h>

#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/goal_cost.h>
#include <itomp_nlp/optimization/velocity_cost.h>
#include <itomp_nlp/optimization/goal_region_cost.h>
#include <itomp_nlp/optimization/repulsive_cost.h>

#include <functional>

#include <iostream>


namespace itomp
{

// dlib bfgs search strategy
class bfgs_search_strategy
{
public:
    bfgs_search_strategy() : been_used(false), been_used_twice(false) {}

    double get_wolfe_rho (
    ) const { return 0.01; }

    double get_wolfe_sigma (
    ) const { return 0.9; }

    unsigned long get_max_line_search_iterations (
    ) const { return 10; }

    template <typename T>
    const dlib::matrix<double,0,1>& get_next_direction (
        const T& x,
        const double ,
        const T& funct_derivative
    )
    {
        if (been_used == false)
        {
            been_used = true;
            H = dlib::identity_matrix<double>(x.size());
        }
        else
        {
            // update H with the BFGS formula from (3.2.12) on page 55 of Fletcher 
            delta = (x-prev_x); 
            gamma = funct_derivative-prev_derivative;

            double dg = dlib::dot(delta,gamma);

            // Try to set the initial value of the H matrix to something reasonable if we are still
            // in the early stages of figuring out what it is.  This formula below is what is suggested
            // in the book Numerical Optimization by Nocedal and Wright in the chapter on Quasi-Newton methods.
            if (been_used_twice == false)
            {
                double gg = dlib::trans(gamma)*gamma;
                if (std::abs(gg) > std::numeric_limits<double>::epsilon())
                {
                    const double temp = dlib::put_in_range(0.01, 100, dg/gg);
                    H = dlib::diagm(dlib::uniform_matrix<double>(x.size(),1, temp));
                    been_used_twice = true;
                }
            }

            Hg = H*gamma;
            gH = dlib::trans(dlib::trans(gamma)*H);
            double gHg = dlib::trans(gamma)*H*gamma;
            if (gHg < std::numeric_limits<double>::infinity() && dg < std::numeric_limits<double>::infinity() &&
                dg != 0)
            {
                H += (1 + gHg/dg)*delta*dlib::trans(delta)/(dg) - (delta*dlib::trans(gH) + Hg*dlib::trans(delta))/(dg);
            }
            else
            {
                H = dlib::identity_matrix<double>(H.nr());
                been_used_twice = false;
            }
        }

        prev_x = x;
        prev_direction = -H*funct_derivative;
        prev_derivative = funct_derivative;
        return prev_direction;
    }

    void reset()
    {
        been_used = false;
        been_used_twice = false;
    }

private:
    bool been_used;
    bool been_used_twice;
    dlib::matrix<double,0,1> prev_x;
    dlib::matrix<double,0,1> prev_derivative;
    dlib::matrix<double,0,1> prev_direction;
    dlib::matrix<double> H;
    dlib::matrix<double,0,1> delta, gamma, Hg, gH;
};


OptimizerThread::OptimizerThread()
{
}

OptimizerThread::~OptimizerThread()
{
}

void OptimizerThread::pushCostFunctionRequest(int id, Cost* cost)
{
    cost_function_request_mutex_.lock();

    CostFunctionRequest request;
    request.id = id;
    request.cost = cost;
    cost_function_requests_.push_back(request);

    cost_function_request_mutex_.unlock();
}

void OptimizerThread::setInitialRobotState(const Eigen::VectorXd& position, const Eigen::VectorXd& velocity)
{
    waypoint_variables_.col(0) = position;
    waypoint_variables_.col(1) = velocity;
}

void OptimizerThread::setRobot(OptimizerRobot* robot)
{
    robot_ = robot->clone();
}

void OptimizerThread::prepare()
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
    interpolated_variables_.setZero();

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

    // dlib variables resize
    dlib_waypoint_variables_.set_size(dof_ * num_waypoints_ * 2);
    dlib_gradient_.set_size(dof_ * num_waypoints_ * 2);

    // best trajectory is the initial trajectory
    storeBestWaypointVariables();
}

void OptimizerThread::start()
{
    thread_stop_requested_ = false;
    move_forward_requests_ = 0;

    thread_ = std::thread( std::bind(&OptimizerThread::threadEnter, this) );
}

void OptimizerThread::stop()
{
    thread_stop_requested_ = true;

    thread_.join();
}

Eigen::MatrixXd OptimizerThread::getBestTrajectory()
{
    mutex_best_waypoint_variables_.lock();
    best_waypoint_variables_ = best_waypoint_variables_pass_;
    best_interpolated_variables_ = best_interpolated_variables_pass_;
    mutex_best_waypoint_variables_.unlock();

    return best_interpolated_variables_;
}

void OptimizerThread::moveForwardOneTimestep()
{
    move_forward_requests_++;
}

void OptimizerThread::threadEnter()
{
    //optimizeGradientDescent();

    optimizeDlib(bfgs_search_strategy(),
                 std::bind(&OptimizerThread::dlibCost, this, std::placeholders::_1),
                 std::bind(&OptimizerThread::dlibDerivative, this, std::placeholders::_1),
                 dlib_waypoint_variables_,
                 -1.);
}

void OptimizerThread::updateWhileOptimizing()
{
    // move forward
    while (move_forward_requests_)
    {
        moveForwardOneTimestepInternal();
        move_forward_requests_--;
    }

    // cost function
    cost_function_request_mutex_.lock();

    if (!cost_function_requests_.empty())
    {
        is_cost_function_updated_ = true;

        for (int i=0; i < cost_function_requests_.size(); i++)
        {
            const int& id = cost_function_requests_[i].id;
            Cost*& cost = cost_function_requests_[i].cost;

            if (cost_id_map_.find(id) == cost_id_map_.end())
            {
                cost_id_map_[id] = cost_functions_.size();
                cost_functions_.push_back(cost);
            }
            else
            {
                delete cost_functions_[ cost_id_map_[id] ];
                cost_functions_[ cost_id_map_[id] ] = cost;
            }
        }

        cost_function_requests_.clear();
    }

    cost_function_request_mutex_.unlock();
}

void OptimizerThread::moveForwardOneTimestepInternal()
{
    int num_shift = (timestep_ + 1e-6) / ((double)trajectory_duration_ / num_waypoints_);
    waypoint_variables_.block(0, 0, dof_, waypoint_variables_.cols() - num_shift * 2) = waypoint_variables_.block(0, 2 * num_shift, dof_, waypoint_variables_.cols() - num_shift * 2); 

    for (int i = waypoint_variables_.cols() - num_shift * 2; i < waypoint_variables_.cols(); i += 2)
    {
        waypoint_variables_.col(i    ) = waypoint_variables_.col(i - 2);
        waypoint_variables_.col(i + 1).setZero();
    }

    // mark if waypoint has been changed
    is_solution_updated_ = true;
}

template <
    typename search_strategy_type,
    typename funct, 
    typename funct_der, 
    typename T
    >
void OptimizerThread::optimizeDlib(
        search_strategy_type search_strategy,
        const funct& f, 
        const funct_der& der, 
        T& x, 
        double min_f
    )
{
    T g, s;

    double f_value = f(x);
    g = der(x);

    while (!thread_stop_requested_)
    {
        // update the trajectory
        updateWhileOptimizing();

        // the waypoint variables might be changed
        bool updated_any = false;
        if (is_solution_updated_)
        {
            updated_any = true;
            is_solution_updated_ = false;

            // update current solution x
            const int num_variables_per_waypoint = dof_ * 2;
            memcpy(waypoint_variables_.data() + num_variables_per_waypoint, x.begin(), num_variables_per_waypoint * num_waypoints_);

            // reset search strategy
            search_strategy.reset();
        }

        if (is_cost_function_updated_)
        {
            updated_any = true;
            is_cost_function_updated_ = false;
            
            // reset search strategy
            search_strategy.reset();
        }

        // if the potential field has been changed, update the initial function value and derivative
        if (updated_any)
        {
            f_value = f(x);
            g = der(x);
        }

        s = search_strategy.get_next_direction(x, f_value, g);

        double alpha = line_search(
                    make_line_search_function(f,x,s, f_value),
                    f_value,
                    make_line_search_function(der,x,s, g),
                    dot(g,s), // compute initial gradient for the line search
                    search_strategy.get_wolfe_rho(), search_strategy.get_wolfe_sigma(), min_f,
                    search_strategy.get_max_line_search_iterations());

        // Take the search step indicated by the above line search
        x += alpha*s;

        // copy x to waypoint variables
        const int num_variables_per_waypoint = dof_ * 2;
        memcpy(waypoint_variables_.data() + num_variables_per_waypoint, x.begin(), sizeof(double) * num_variables_per_waypoint * num_waypoints_);

        // store best waypoint
        storeBestWaypointVariables();
        
        // DEBUG: print cost functions
        /*
        optimizationPrecomputation();
        for (int i=0; i<cost_functions_.size(); i++)
        {
            const double c = cost_functions_[i]->cost();
            printf("%.9lf ", c);
        }
        printf("\n");
        */
    }
}

double OptimizerThread::dlibCost(const column_vector& x)
{
    // copy x to waypoint variables
    // skip the first two columns which are initial state
    const int num_variables_per_waypoint = dof_ * 2;
    memcpy(waypoint_variables_.data() + num_variables_per_waypoint, x.begin(), sizeof(double) * num_variables_per_waypoint * num_waypoints_);

    // precomputation is done in cost function
    optimizationPrecomputation();

    return cost();
}

OptimizerThread::column_vector OptimizerThread::dlibDerivative(const column_vector& x)
{
    // assumed that precomputation for x is previously done
    computeGradientChainRule();

    // copy gradient to dlib gradient
    const int num_variables_per_waypoint = dof_ * 2;
    memcpy(dlib_gradient_.begin(), gradient_.data(), sizeof(double) * num_variables_per_waypoint * num_waypoints_);

    return dlib_gradient_;
}

void OptimizerThread::optimizeGradientDescent()
{
    int iterations = 0;

    const double alpha = 0.001;

    while (!thread_stop_requested_)
    {
        updateWhileOptimizing();
        
        optimizationPrecomputation();

        // cost function
        const double f = cost();
        
        // DEBUG: print iteration and costs (slow)
        printf("iteration %5d: %lf\n", iterations, f);
        
        /*
        // DEBUG: endeffector (link 7) position print
        Eigen::Vector3d e = (*forward_kinematics_robots_.rbegin())->getLinkWorldTransform(7) * Eigen::Vector3d(0.1, 0, 0);
        printf("link 7 position: %lf %lf %lf\n", e(0), e(1), e(2));
        */

        // simple gradient descent update
        //computeGradientDirect();
        computeGradientChainRule();

        // simple gradient descent
        waypoint_variables_.block(0, 2, dof_, num_waypoints_ * 2) -= alpha * gradient_;

        // update as the best trajectory
        storeBestWaypointVariables();

        iterations++;
    }
}

void OptimizerThread::storeBestWaypointVariables()
{
    mutex_best_waypoint_variables_.lock();
    best_waypoint_variables_pass_ = waypoint_variables_;
    best_interpolated_variables_pass_ = interpolated_variables_;
    mutex_best_waypoint_variables_.unlock();
}

double OptimizerThread::cost()
{
    double cost = 0.;

    for (int i=0; i<cost_functions_.size(); i++)
        cost += cost_functions_[i]->cost();

    return cost;
}

double OptimizerThread::cost(int interpolation_idx)
{
    double cost = 0.;

    for (int i=0; i<cost_functions_.size(); i++)
        cost += cost_functions_[i]->cost(interpolation_idx);

    return cost;
}

void OptimizerThread::computeGradientDirect()
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
            optimizationPrecomputation();

            gradient_(j, i-2) = (forward_cost - backward_cost) / (2. * delta);
        }
    }

    // restore interpolation variables modified by central difference method
    optimizationPrecomputation();
}

void OptimizerThread::computeGradientChainRule()
{
    gradient_.setZero();
    
    Eigen::MatrixXd chain(dof_, 2);

    // central difference
    const double delta = 0.0001;
    for (int i=0; i < num_waypoints_; i++)
    {
        for (int j = 1; j <= num_waypoint_interpolations_ + 1; j++)
        {
            const int interpolation_idx = i * (num_waypoint_interpolations_ + 1) + j;
            const int col_position = interpolation_idx * 2;
            const int col_velocity = interpolation_idx * 2 + 1;

            for (int k=0; k<dof_; k++)
            {
                for (int l=0; l<2; l++)
                {
                    const double original_value = interpolated_variables_( k, col_position + l );

                    // forward cost
                    interpolated_variables_( k, col_position + l ) += delta;

                    // local precomputation
                    forward_kinematics_robots_[interpolation_idx]->setPositions ( interpolated_variables_.col( col_position ) );
                    forward_kinematics_robots_[interpolation_idx]->setVelocities( interpolated_variables_.col( col_velocity ) );
                    forward_kinematics_robots_[interpolation_idx]->forwardKinematics();

                    const double forward_cost = cost(interpolation_idx);
                    
                    // backward cost
                    interpolated_variables_( k, col_position + l ) -= 2. * delta;

                    // local precomputation
                    forward_kinematics_robots_[interpolation_idx]->setPositions ( interpolated_variables_.col( col_position ) );
                    forward_kinematics_robots_[interpolation_idx]->setVelocities( interpolated_variables_.col( col_velocity ) );
                    forward_kinematics_robots_[interpolation_idx]->forwardKinematics();

                    const double backward_cost = cost(interpolation_idx);

                    // restore
                    interpolated_variables_( k, col_position + l ) = original_value;
                    forward_kinematics_robots_[interpolation_idx]->setPositions ( interpolated_variables_.col( col_position ) );
                    forward_kinematics_robots_[interpolation_idx]->setVelocities( interpolated_variables_.col( col_velocity ) );
                    forward_kinematics_robots_[interpolation_idx]->forwardKinematics();

                    chain(k, l) = (forward_cost - backward_cost) / (2. * delta);
                }
            }

            // the derivative is chained with hermite spline coefficients
            if (i != 0)
            {
                gradient_.col(i*2-2) += chain * Eigen::Vector2d(interpolation_coefficients_(0, j*2), interpolation_coefficients_(0, j*2+1));
                gradient_.col(i*2-1) += chain * Eigen::Vector2d(interpolation_coefficients_(1, j*2), interpolation_coefficients_(1, j*2+1));
            }
            gradient_.col(i*2  ) += chain * Eigen::Vector2d(interpolation_coefficients_(2, j*2), interpolation_coefficients_(2, j*2+1));
            gradient_.col(i*2+1) += chain * Eigen::Vector2d(interpolation_coefficients_(3, j*2), interpolation_coefficients_(3, j*2+1));
        }
    }

    // restore interpolation variables modified by central difference method
    optimizationPrecomputation();
}

void OptimizerThread::optimizationPrecomputation()
{
    interpolate();
    forwardKinematics();
}

void OptimizerThread::interpolate()
{
    for (int i=0; i<num_waypoints_; i++)
        interpolated_variables_.block(0, 2 * i * (num_waypoint_interpolations_ + 1), dof_, (num_waypoint_interpolations_ + 2) * 2) = waypoint_variables_.block(0, i * 2, dof_, 4) * interpolation_coefficients_;
}

void OptimizerThread::forwardKinematics()
{
    for (int i=0; i<forward_kinematics_robots_.size(); i++)
    {
        forward_kinematics_robots_[i]->setPositions (interpolated_variables_.col(2*i  ));
        forward_kinematics_robots_[i]->setVelocities(interpolated_variables_.col(2*i+1));

        forward_kinematics_robots_[i]->forwardKinematics();
    }
}

}
