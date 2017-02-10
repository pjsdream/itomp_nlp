#include <itomp_nlp/optimization/static_obstacle.h>


namespace itomp
{

StaticObstacle::StaticObstacle()
{
}

void StaticObstacle::addShape(Shape* shape)
{
    shapes_.push_back(shape);
}

}