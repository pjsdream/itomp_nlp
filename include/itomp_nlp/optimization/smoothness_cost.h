#ifndef ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H
#define ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H


#include <itomp_nlp/optimization/cost.h>


namespace itomp_optimization
{

class SmoothnessCost : public Cost
{
public:

    SmoothnessCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new SmoothnessCost(*this);
    }

    virtual double cost();
    virtual double cost(int idx);

private:

    double f(double x);
};

}


#endif // ITOMP_OPTIMIZATION_SMOOTHNESS_COST_H