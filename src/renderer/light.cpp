#include <itomp_nlp/renderer/light.h>


namespace itomp_renderer
{

Light::Light(const Eigen::Vector3d& position, const Eigen::Vector3d& color)
    : position_(position)
    , color_(color)
{
}

}
