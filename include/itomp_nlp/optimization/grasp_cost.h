#ifndef ITOMP_OPTIMIZATION_PICKUP_COST_H
#define ITOMP_OPTIMIZATION_PICKUP_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <itomp_nlp/optimization/scene.h>


namespace itomp
{

class GraspCost : public Cost
{
public:

    GraspCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new GraspCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

private:

};

}


#endif // ITOMP_OPTIMIZATION_PICKUP_COST_H