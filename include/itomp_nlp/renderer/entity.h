#ifndef ITOMP_RENDERER_ENTITY_H
#define ITOMP_RENDERER_ENTITY_H


#include <itomp_nlp/renderer/object.h>

#include <Eigen/Dense>


namespace itomp
{

class Entity
{
public:
    
    Entity(Object* model);
    Entity(Object* model, const Eigen::Affine3d& transformation);

    inline Object* getObject()
    {
        return object_;
    }

    inline const Eigen::Affine3d& getTransformation() const
    {
        return transformation_;
    }

    inline Eigen::Affine3d& getTransformation()
    {
        return transformation_;
    }

    inline void setTransformation(const Eigen::Affine3d& transformation)
    {
        transformation_ = transformation;
    }

private:

    Object* object_;
    Eigen::Affine3d transformation_;
};

}


#endif // ITOMP_RENDERER_ENTITY_H