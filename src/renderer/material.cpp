#include <itomp_nlp/renderer/material.h>

#include <lodepng.h>


namespace itomp
{

Material::Material()
    : ambient_color_(Eigen::Vector4f(0, 0, 0, 1))
    , diffuse_color_(Eigen::Vector4f(0.5, 0.5, 0.5, 1))
    , specular_color_(Eigen::Vector4f(0, 0, 0, 1))
    , has_diffuse_texture_(false)
    , shininess_(50.)
{
}

}
