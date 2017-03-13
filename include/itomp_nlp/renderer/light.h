#ifndef ITOMP_RENDERER_LIGHT_H
#define ITOMP_RENDERER_LIGHT_H


#include <Eigen/Dense>


namespace itomp
{

class Light
{
public:

    Light(const Eigen::Vector3d& position = Eigen::Vector3d::Zero());

    inline const Eigen::Vector3d& getPosition() const
    {
        return position_;
    }

    inline const Eigen::Vector4f& getDiffuseColor() const
    {
        return diffuse_color_;
    }

    inline const Eigen::Vector4f& getSpecularColor() const
    {
        return specular_color_;
    }

    inline void setPosition(const Eigen::Vector3d& position)
    {
        position_ = position;
    }

    inline void setDiffuseColor(const Eigen::Vector4f& color)
    {
        diffuse_color_ = color;
    }

    inline void setSpecularColor(const Eigen::Vector4f& color)
    {
        specular_color_ = color;
    }

private:

    Eigen::Vector3d position_;
    Eigen::Vector4f diffuse_color_;
    Eigen::Vector4f specular_color_;
};

}


#endif // ITOMP_RENDERER_LIGHT_H