#ifndef ITOMP_OPTIMIZATION_COST_H
#define ITOMP_OPTIMIZATION_COST_H


#include <Eigen/Dense>


namespace itomp_optimization
{

class OptimizerThread;

// zero cost
class Cost
{
public:

    Cost(OptimizerThread& optimizer, double weight = 1.);

    inline virtual Cost* clone() const
    {
        return new Cost(*this);
    }

    inline void setWeight(double weight)
    {
        weight_ = weight;
    }

    virtual double cost();
    virtual double cost(int idx);

protected:

    OptimizerThread& optimizer_;
    double weight_;
};

}


#endif // ITOMP_OPTIMIZATION_COST_H