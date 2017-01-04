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

    GoalCost(Optimizer& optimizer, double weight);
    
    virtual double cost();

    void addGoalPosition(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& goal_position);

private:

    std::vector<Goal> goal_positions_;
};

}


#endif // ITOMP_OPTIMIZATION_GOAL_COST_H