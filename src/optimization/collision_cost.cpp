#include <itomp_nlp/optimization/collision_cost.h>
#include <itomp_nlp/optimization/optimizer_thread.h>


namespace itomp
{

CollisionCost::CollisionCost(OptimizerThread& optimizer, Scene* scene, double weight)
    : Cost(optimizer, weight)
    , scene_(scene)
{
}
    
double CollisionCost::cost()
{
    double c = 0.;

    for (int i=0; i<optimizer_.forward_kinematics_robots_.size(); i++)
        c += cost(i);

    return c;
}

double CollisionCost::cost(int idx)
{
    double cost = 0.;

    OptimizerRobot* robot = optimizer_.forward_kinematics_robots_[idx];

    for (int i=0; i<robot->getNumLinks(); i++)
    {
        const std::vector<Shape*>& robot_shapes = robot->getCollisionShapes(i);
        for (int j=0; j<robot_shapes.size(); j++)
        {
            Shape* robot_shape = robot_shapes[j];

            for (int k=0; k<static_shapes_.size(); k++)
                cost += robot_shape->getPenetrationDepth(static_shapes_[k]);

            for (int k=0; k<dynamic_shapes_[idx].size(); k++)
                cost += robot_shape->getPenetrationDepth(dynamic_shapes_[idx][k]);
        }
    }

    return cost * weight_;
}

void CollisionCost::updateSceneObstacles()
{
    scene_->lock();

    const std::vector<StaticObstacle*> static_obstacles = scene_->getStaticObstacles();

    for (int i=0; i<static_shapes_.size(); i++)
        delete static_shapes_[i];
    static_shapes_.clear();

    for (int i=0; i<static_obstacles.size(); i++)
    {
        const std::vector<Shape*>& static_shapes = static_obstacles[i]->getShapes();
        for (int j=0; j<static_shapes.size(); j++)
            static_shapes_.push_back(static_shapes[j]->clone());
    }

    const std::vector<DynamicObstacle*> dynamic_obstacles = scene_->getDynamicObstacles();

    dynamic_shapes_.resize( optimizer_.getNumInterpolatedVariables() );
    for (int i=0; i<optimizer_.getNumInterpolatedVariables(); i++)
    {
        for (int j=0; j<dynamic_shapes_[i].size(); j++)
            delete dynamic_shapes_[i][j];
        dynamic_shapes_[i].clear();

        for (int j=0; j<dynamic_obstacles.size(); j++)
        {
            const double time = optimizer_.getInterpolationIndexToTime(i);
            const std::vector<Shape*> dynamic_shapes = dynamic_obstacles[j]->getShapes(time);

            for (int k=0; k<dynamic_shapes.size(); k++)
                dynamic_shapes_[i].push_back(dynamic_shapes[k]->clone());
        }
    }

    scene_->unlock();
}

}
