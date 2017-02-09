#include <itomp_nlp/optimization/scene.h>


namespace itomp
{

Scene::Scene()
{
}

void Scene::addStaticObstacle(StaticObstacle* obstacle)
{
    std::lock_guard<std::mutex> lock(mutex_);
    static_obstacles_.push_back(obstacle);
}

void Scene::addDynamicObstacle(DynamicObstacle* obstacle)
{
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_obstacles_.push_back(obstacle);
}

void Scene::lock()
{
    mutex_.lock();
}

void Scene::unlock()
{
    mutex_.unlock();
}

}