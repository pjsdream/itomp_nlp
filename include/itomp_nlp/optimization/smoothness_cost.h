#ifndef ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H
#define ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H


#include <itomp_nlp/optimization/cost.h>


namespace itomp_optimization
{

class SmoothnessCost : public Cost
{
public:

    SmoothnessCost(Optimizer& optimizer, double weight);

    virtual double cost();

private:
};

}


#endif // ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H