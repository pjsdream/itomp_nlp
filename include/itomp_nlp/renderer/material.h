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

    inline const Eigen::Vector4f& getAmbientColor() const
    {
        return ambient_color_;
    }

    inline const Eigen::Vector4f& getDiffuseColor() const
    {
        return diffuse_color_;
    }

    inline const Eigen::Vector4f& getSpecularColor() const
    {
        return specular_color_;
    }

    inline void setAmbientColor(const Eigen::Vector4f& color)
    {
        ambient_color_ = color;
    }

    inline void setDiffuseColor(const Eigen::Vector4f& color)
    {
        has_diffuse_texture_ = false;
        diffuse_color_ = color;
    }

    inline void setSpecularColor(const Eigen::Vector4f& color)
    {
        specular_color_ = color;
    }

    inline bool hasDiffuseTexture() const
    {
        return has_diffuse_texture_;
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
        has_diffuse_texture_ = true;
        diffuse_texture_ = texture;
    }

    inline Texture* getDiffuseTexture()
    {
        return diffuse_texture_;
    }

private:
    
    Eigen::Vector4f ambient_color_;
    Eigen::Vector4f diffuse_color_;
    Eigen::Vector4f specular_color_;

    bool has_diffuse_texture_;

    Texture* diffuse_texture_;

    float shininess_;
};

}


#endif // ITOMP_RENDERER_MATERIAL_H