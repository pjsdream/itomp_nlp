#ifndef ITOMP_OPTIMIZATION_REPULSIVE_COST_H
#define ITOMP_OPTIMIZATION_REPULSIVE_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <vector>

#include <Eigen/Dense>


namespace itomp_optimization
{

class RepulsiveCost : public Cost
{
private:

    struct Repulsion
    {
        int link_id;
        Eigen::Vector3d translation;
        Eigen::Vector3d repulsion_center;
        double distance;
    };

public:

    RepulsiveCost(Optimizer& optimizer, double weight);
    
    virtual double cost();
    virtual double cost(int idx);

    void addRepulsion(int link_id, const Eigen::Vector3d& translation, const Eigen::Vector3d& repulsion_center, double distance);

private:

    std::vector<Repulsion> repulsions_;

    double f(double x, double d);
};

}


#endif // ITOMP_OPTIMIZATION_REPULSIVE_COST_H