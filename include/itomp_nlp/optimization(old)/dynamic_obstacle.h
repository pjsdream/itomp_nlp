#ifndef ITOMP_OPTIMIZATION_DYNAMIC_OBSTACLE_H
#define ITOMP_OPTIMIZATION_DYNAMIC_OBSTACLE_H


#include <itomp_nlp/shape/shape.h>

#include <vector>


namespace itomp
{

class DynamicObstacle
{
public:

    DynamicObstacle();

    virtual std::vector<Shape*> getShapes(double t) = 0;

private:
};

}


#endif // ITOMP_OPTIMIZATION_DYNAMIC_OBSTACLE_H