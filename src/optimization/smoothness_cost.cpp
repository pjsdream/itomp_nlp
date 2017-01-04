#include <itomp_nlp/optimization/smoothness_cost.h>
#include <itomp_nlp/optimization/optimizer.h>


namespace itomp_optimization
{

SmoothnessCost::SmoothnessCost(Optimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}

double SmoothnessCost::cost()
{
    double cost = 0.;

    // position
    for (int i=0; i<optimizer_.interpolated_variables_.cols() - 2; i += 2)
    {
        for (int j=0; j<optimizer_.interpolated_variables_.rows(); j++)
        {
            const double& q0 = optimizer_.interpolated_variables_(j, i);
            const double& q1 = optimizer_.interpolated_variables_(j, i + 2);

            cost += (q1-q0) * (q1-q0);
        }
    }

    // TODO: velocity

    return cost * weight_;
}

}
