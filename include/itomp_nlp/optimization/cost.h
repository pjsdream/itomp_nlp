#ifndef ITOMP_OPTIMIZATION_COST_H
#define ITOMP_OPTIMIZATION_COST_H


#include <Eigen/Dense>


namespace itomp_optimization
{

class OptimizerThread;

class Cost
{
public:

    Cost(OptimizerThread& optimizer, double weight = 1.);

    virtual double cost() = 0;
    virtual double cost(int idx) = 0;

protected:

    OptimizerThread& optimizer_;
    double weight_;
};

}


#endif // ITOMP_OPTIMIZATION_COST_H