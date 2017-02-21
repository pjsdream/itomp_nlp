#include <itomp_nlp/optimization/grasp_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp
{

GraspCost::GraspCost(OptimizerThread& optimizer, double weight)
    : Cost(optimizer, weight)
{
}
    
double GraspCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double GraspCost::cost(int idx)
{
    double cost = 0.;

    return cost * weight_;
}

}
