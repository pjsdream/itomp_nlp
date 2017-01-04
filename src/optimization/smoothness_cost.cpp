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
    return 0;
}

}
