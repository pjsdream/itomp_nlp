#ifndef ITOMP_OPTIMIZATION_GOAL_UPVECTOR_COST_H
#define ITOMP_OPTIMIZATION_GOAL_UPVECTOR_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class GoalUpvectorCost : public Cost
{
private:

    struct Goal
    {
        int link_id;
        Eigen::Vector3d goal_upvector;
    };

public:

    GoalUpvectorCost(OptimizerThread& optimizer, double weight);

    inline virtual Cost* clone() const
    {
        return new GoalUpvectorCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

    void setGoalUpvector(int link_id, const Eigen::Vector3d& goal_upvector);

    inline const Eigen::Vector3d& getGoalUpvector() const
    {
        return goal_.goal_upvector;
    }

private:

    Goal goal_;
};

}


#endif // ITOMP_OPTIMIZATION_GOAL_UPVECTOR_COST_H