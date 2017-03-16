#ifndef ITOMP_RENDERER_MATERIAL_H
#define ITOMP_RENDERER_MATERIAL_H


#include <Eigen/Dense>

#include <itomp_nlp/renderer/texture.h>


namespace itomp
{

class Material
{
public:

    Material();

    inline const Eigen::Vector3f& getAmbient() const
    {
        return ambient_;
    }

    inline const Eigen::Vector3f& getDiffuse() const
    {
        return diffuse_;
    }

    inline const Eigen::Vector3f& getSpecular() const
    {
        return specular_;
    }

    inline void setAmbient(const Eigen::Vector3f& color)
    {
        ambient_ = color;
    }

    inline void setDiffuse(const Eigen::Vector3f& color)
    {
        has_diffuse_texture_ = false;
        diffuse_ = color;
    }

    inline void setSpecular(const Eigen::Vector3f& color)
    {
        specular_ = color;
    }

    inline bool hasDiffuseTexture() const
    {
        return diffuse_texture_ != 0;
    }

    inline float getShininess() const
    {
        return shininess_;
    }

    inline void setShininess(float shininess)
    {
        shininess_ = shininess;
    }

    inline void setDiffuseTexture(Texture* texture)
    {
        diffuse_texture_ = texture;
    }

    inline Texture* getDiffuseTexture() const
    {
        return diffuse_texture_;
    }

private:
    
    Eigen::Vector3f ambient_;
    Eigen::Vector3f diffuse_;
    Eigen::Vector3f specular_;

    bool alpha_;

    bool has_diffuse_texture_;

    Texture* diffuse_texture_;

    float shininess_;
};

}


#endif // ITOMP_RENDERER_MATERIAL_H