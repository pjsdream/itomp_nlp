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
            {
                const double d = robot_shape->getPenetrationDepth(static_shapes_[k]);
                cost += d*d;
            }

            for (int k=0; k<dynamic_shapes_[idx].size(); k++)
            {
                const double d = robot_shape->getPenetrationDepth(dynamic_shapes_[idx][k]);
                cost += d*d;
            }
        }
    }

    // self collision cost
    for (int i=0; i<robot->getNumLinks(); i++)
    {
        const std::vector<Shape*>& robot_shapes_1 = robot->getCollisionShapes(i);
        for (int j=i+1; j<robot->getNumLinks(); j++)
        {
            if (robot->needSelfCollisionChecking(i, j))
            {
                const std::vector<Shape*>& robot_shapes_2 = robot->getCollisionShapes(j);

                for (int k=0; k<robot_shapes_1.size(); k++)
                {
                    Shape* robot_shape_1 = robot_shapes_1[k];
                    for (int l=0; l<robot_shapes_2.size(); l++)
                    {
                        Shape* robot_shape_2 = robot_shapes_2[l];

                        const double d = robot_shape_1->getPenetrationDepth(robot_shape_2);
                        cost += d*d;
                    }
                }
            }
        }
    }

    return cost * weight_;
}

void CollisionCost::updateSceneObstacles()
{
    scene_->lock();

    const std::vector<StaticObstacle*> static_obstacles = scene_->getStaticObstacles();

    static_shapes_.clear();

    for (int i=0; i<static_obstacles.size(); i++)
    {
        const std::vector<Shape*>& static_shapes = static_obstacles[i]->getShapes();
        for (int j=0; j<static_shapes.size(); j++)
            static_shapes_.push_back(static_shapes[j]);
    }

    const std::vector<DynamicObstacle*> dynamic_obstacles = scene_->getDynamicObstacles();

    dynamic_shapes_.resize( optimizer_.getNumInterpolatedVariables() );
    for (int i=0; i<optimizer_.getNumInterpolatedVariables(); i++)
    {
        dynamic_shapes_[i].clear();

        for (int j=0; j<dynamic_obstacles.size(); j++)
        {
            const double time = optimizer_.getInterpolationIndexToTime(i);
            const std::vector<Shape*> dynamic_shapes = dynamic_obstacles[j]->getShapes(time);

            for (int k=0; k<dynamic_shapes.size(); k++)
                dynamic_shapes_[i].push_back(dynamic_shapes[k]);
        }
    }

    scene_->unlock();
}

}
