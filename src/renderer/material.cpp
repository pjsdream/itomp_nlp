#include <renderer/material.h>


namespace itomp_renderer
{

Material::Material(const Eigen::Vector3d& ambient_color, const Eigen::Vector3d& specular_color)
    : ambient_color_(ambient_color)
    , diffuse_color_(Eigen::Vector3d::Zero())
    , specular_color_(specular_color)
{
}

}
