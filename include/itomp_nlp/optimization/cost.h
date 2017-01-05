#ifndef ITOMP_OPTIMIZATION_COST_H
#define ITOMP_OPTIMIZATION_COST_H


#include <Eigen/Dense>


namespace itomp_optimization
{

class Optimizer;

class Cost
{
public:

    Cost(Optimizer& optimizer, double weight = 1.);

    virtual double cost() = 0;
    virtual double cost(int idx) = 0;

protected:

    Optimizer& optimizer_;
    double weight_;
};

}


#endif // ITOMP_OPTIMIZATION_COST_H