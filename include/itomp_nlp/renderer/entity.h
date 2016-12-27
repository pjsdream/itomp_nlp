#ifndef ITOMP_RENDERER_ENTITY_H
#define ITOMP_RENDERER_ENTITY_H


#include <itomp_nlp/renderer/textured_model.h>

#include <Eigen/Dense>


namespace itomp_renderer
{

class Entity
{
public:
    
    Entity(TexturedModel* model, const Eigen::Affine3d& transformation);

    inline TexturedModel* getModel()
    {
        return model_;
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

    TexturedModel* model_;
    Eigen::Affine3d transformation_;
};

}


#endif // ITOMP_RENDERER_ENTITY_H