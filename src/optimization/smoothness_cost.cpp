#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp_optimization
{

SmoothnessCost::SmoothnessCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double SmoothnessCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double SmoothnessCost::cost(int idx)
{
    double cost = 0.;

    // velocity
    for (int j=0; j<optimizer_.interpolated_variables_.rows(); j++)
    {
        const double& q = optimizer_.interpolated_variables_(j, idx*2+1);

        cost += f(q);
    }

    return cost * weight_;
}

double SmoothnessCost::f(double x)
{
    x = std::abs(x);
    return x < 0.01 ? 0. : x - 0.01;
}

}
