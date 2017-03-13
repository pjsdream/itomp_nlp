#ifndef ITOMP_OPTIMIZATION_SCENE_H
#define ITOMP_OPTIMIZATION_SCENE_H


#include <itomp_nlp/optimization/static_obstacle.h>
#include <itomp_nlp/optimization/dynamic_obstacle.h>

#include <thread>
#include <mutex>

#include <vector>


namespace itomp
{

class Scene
{
public:

    Scene();

    void addStaticObstacle(StaticObstacle* obstacle);
    void addDynamicObstacle(DynamicObstacle* obstacle);

    inline const std::vector<StaticObstacle*> getStaticObstacles()
    {
        return static_obstacles_;
    }

    inline const std::vector<DynamicObstacle*> getDynamicObstacles()
    {
        return dynamic_obstacles_;
    }

    void lock();
    void unlock();

private:

    std::mutex mutex_;

    std::vector<StaticObstacle*> static_obstacles_;
    std::vector<DynamicObstacle*> dynamic_obstacles_;
};

}


#endif // ITOMP_OPTIMIZATION_SCENE_H