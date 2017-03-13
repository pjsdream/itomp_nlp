#include <itomp_nlp/renderer/light.h>


namespace itomp
{

Light::Light(const Eigen::Vector3d& position)
    : position_(position)
    , diffuse_color_(Eigen::Vector4f(1, 1, 1, 1))
    , specular_color_(Eigen::Vector4f(0, 0, 0, 1))
{
}

}
