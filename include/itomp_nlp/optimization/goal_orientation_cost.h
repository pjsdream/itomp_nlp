#ifndef ITOMP_OPTIMIZATION_GOAL_ORIENTATION_COST_H
#define ITOMP_OPTIMIZATION_GOAL_ORIENTATION_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class GoalOrientationCost : public Cost
{
private:

    struct Goal
    {
        int link_id;
        Eigen::Quaterniond goal_orientation;
    };

public:

    GoalOrientationCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new GoalOrientationCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

    void setGoalOrientation(int link_id, const Eigen::Quaterniond& goal_orientation);

    inline const Eigen::Quaterniond& getGoalOrientation() const
    {
        return goal_.goal_orientation;
    }

private:

    Goal goal_;
};

}


#endif // ITOMP_OPTIMIZATION_GOAL_ORIENTATION_COST_H