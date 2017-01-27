#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp
{

CollisionCost::CollisionCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}
    
double CollisionCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double CollisionCost::cost(int idx)
{
    // TODO
    return 0.;
}

}
