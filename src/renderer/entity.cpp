#include <itomp_nlp/renderer/entity.h>


namespace itomp_renderer
{

Entity::Entity(TexturedModel* model, const Eigen::Affine3d& transformation)
    : model_(model), transformation_(transformation)
{
}

}
