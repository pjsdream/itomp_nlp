#include <itomp_nlp/renderer/material.h>

#include <lodepng.h>


namespace itomp
{

Material::Material()
    : ambient_(0.5, 0.5, 0.5)
    , diffuse_(0.5, 0.5, 0.5)
    , specular_(0.5, 0.5, 0.5)
    , shininess_(32.)
    , diffuse_texture_(0)
    , alpha_(1)
{
}

}
