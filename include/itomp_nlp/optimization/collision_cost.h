#ifndef ITOMP_OPTIMIZATION_COLLISION_COST_H
#define ITOMP_OPTIMIZATION_COLLISION_COST_H


#include <itomp_nlp/optimization/cost.h>

#include <itomp_nlp/optimization/scene.h>


namespace itomp
{

class CollisionCost : public Cost
{
public:

    CollisionCost(OptimizerThread& optimizer, Scene* scene, double weight);

    inline virtual Cost* clone() const
    {
        return new CollisionCost(*this);
    }
    
    virtual double cost();
    virtual double cost(int idx);

    void setScene(Scene* scene);
    void updateSceneObstacles();

private:

    Scene* scene_;

    std::vector<Shape*> static_shapes_;
    std::vector<std::vector<Shape*> > dynamic_shapes_;
};

}


#endif // ITOMP_OPTIMIZATION_COLLISION_COST_H