#include <itomp_nlp/renderer/entity.h>


namespace itomp_renderer
{

Entity::Entity(Object* object)
    : object_(object)
    , transformation_(Eigen::Affine3d::Identity())
{
}

Entity::Entity(Object* object, const Eigen::Affine3d& transformation)
    : object_(object)
    , transformation_(transformation)
{
}

}