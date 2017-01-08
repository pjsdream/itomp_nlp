#ifndef ITOMP_OPTIMIZATION_GOAL_COST_H
#define ITOMP_OPTIMIZATION_GOAL_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp_optimization
{

class GoalCost : public Cost
{
private:

    struct Goal
    {
        int link_id;
        Eigen::Vector3d translation;
        Eigen::Vector3d goal_position;
    };

public:

    GoalCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new GoalCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

    void addGoalPosition(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position);

private:

    std::vector<Goal> goal_positions_;

    double f(double x);
};

}


#endif // ITOMP_OPTIMIZATION_GOAL_COST_H