#include <itomp_nlp/renderer/light.h>


namespace itomp
{

Light::Light(const Eigen::Vector3d& position)
    : light_type_(LightType::Point)
    , position_(position)
    , ambient_(1, 1, 1)
    , diffuse_(1, 1, 1)
    , specular_(0, 0, 0)
    , attenuation_(1, 0, 0)
{
}

}
