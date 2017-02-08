#include <itomp_nlp/optimization/scene.h>


namespace itomp
{

Scene::Scene()
{
}

void Scene::addStaticObstacle(StaticObstacle* obstacle)
{
    static_obstacles_.push_back(obstacle);
}

void Scene::addDynamicObstacle(DynamicObstacle* obstacle)
{
    dynamic_obstacles_.push_back(obstacle);
}

}