#include <itomp_nlp/renderer/light.h>


namespace itomp_renderer
{

Light::Light(const Eigen::Vector3d& position)
    : position_(position)
    , color_(Eigen::Vector4f(0, 0, 0, 1))
{
}

}
