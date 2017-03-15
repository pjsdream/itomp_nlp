#ifndef ITOMP_OPTIMIZATION_VELOCITY_COST_H
#define ITOMP_OPTIMIZATION_VELOCITY_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class VelocityCost : public Cost
{
private:

    struct GoalVelocity
    {
        int link_id;
        Eigen::Vector3d velocity;
    };

public:

    VelocityCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new VelocityCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

    void setGoalVelocity(int link_id, const Eigen::Vector3d& velocity);

private:

    GoalVelocity goal_;

    double f(double x);
};

}


#endif // ITOMP_OPTIMIZATION_VELOCITY_COST_H