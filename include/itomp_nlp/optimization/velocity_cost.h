#ifndef ITOMP_OPTIMIZATION_VELOCITY_COST_H
#define ITOMP_OPTIMIZATION_VELOCITY_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp_optimization
{

class VelocityCost : public Cost
{
private:

    struct GoalVelocity
    {
        int link_id;
        Eigen::Vector3d translation;
        Eigen::Vector3d velocity;
    };

public:

    VelocityCost(Optimizer& optimizer, double weight);
    
    virtual double cost();

    void addGoalVelocity(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& velocity);

private:

    std::vector<GoalVelocity> velocities_;
};

}


#endif // ITOMP_OPTIMIZATION_VELOCITY_COST_H