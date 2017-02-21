#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp
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
        const double q = optimizer_.interpolated_variables_(j, idx*2+1);
        cost += q*q;
    }

    return cost * weight_;
}

}
