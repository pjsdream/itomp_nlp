#ifndef ITOMP_OPTIMIZATION_GOAL_REGION_COST_H
#define ITOMP_OPTIMIZATION_GOAL_REGION_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp_optimization
{

class GoalRegionCost : public Cost
{
private:

    struct GoalPlane
    {
        int link_id;
        Eigen::Vector3d translation;
        Eigen::Vector4d plane;
    };

public:

    GoalRegionCost(Optimizer& optimizer, double weight);
    
    virtual double cost();
    virtual double cost(int idx);

    void addGoalRegionPlane(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector4d& plane);

private:

    std::vector<GoalPlane> goal_planes_;

    double f(double x);
};

}


#endif // ITOMP_OPTIMIZATION_GOAL_REGION_COST_H