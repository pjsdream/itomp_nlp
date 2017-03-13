#ifndef ITOMP_OPTIMIZATION_STATIC_OBSTACLE_H
#define ITOMP_OPTIMIZATION_STATIC_OBSTACLE_H


#include <itomp_nlp/shape/shape.h>

#include <vector>


namespace itomp
{

class StaticObstacle
{
public:

    StaticObstacle();

    void addShape(Shape* shape);

    inline const std::vector<Shape*>& getShapes() const
    {
        return shapes_;
    }

private:

    std::vector<Shape*> shapes_;
};

}


#endif // ITOMP_OPTIMIZATION_STATIC_OBSTACLE_H