#ifndef ITOMP_OPTIMIZATION_COLLISION_COST_H
#define ITOMP_OPTIMIZATION_COLLISION_COST_H


#include <itomp_nlp/optimization/cost.h>


namespace itomp_optimization
{

class CollisionCost : public Cost
{
public:

    CollisionCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new CollisionCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

private:
};

}


#endif // ITOMP_OPTIMIZATION_COLLISION_COST_H