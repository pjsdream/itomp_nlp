#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/optimizer.h>


namespace itomp_optimization
{

CollisionCost::CollisionCost(Optimizer& optimizer, double weight)
    : Cost(optimizer, weight)
{
}
    
double CollisionCost::cost()
{
    return 0.;
}

}
