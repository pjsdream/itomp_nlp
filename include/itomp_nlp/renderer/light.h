#ifndef ITOMP_RENDERER_LIGHT_H
#define ITOMP_RENDERER_LIGHT_H


#include <Eigen/Dense>


namespace itomp
{

class Light
{
public:

    enum LightType
    {
        Directional = 0,
        Point,
    };

public:

    Light(const Eigen::Vector3d& position = Eigen::Vector3d::Zero());

    inline void setDirectional()
    {
        light_type_ = Directional;
    }

    inline void setPoint()
    {
        light_type_ = Point;
    }

    inline bool isDirectional() const
    {
        return light_type_ == LightType::Directional;
    }

    inline bool isPoint() const
    {
        return light_type_ == LightType::Point;
    }

    inline const Eigen::Vector3d& getPosition() const
    {
        return position_;
    }

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

    inline const Eigen::Vector3f& getAttenuation() const
    {
        return attenuation_;
    }

    inline void setPosition(const Eigen::Vector3d& position)
    {
        position_ = position;
    }

    inline void setAmbient(const Eigen::Vector3f& color)
    {
        ambient_ = color;
    }

    inline void setDiffuse(const Eigen::Vector3f& color)
    {
        diffuse_ = color;
    }

    inline void setSpecular(const Eigen::Vector3f& color)
    {
        specular_ = color;
    }

    inline void setAttenuation(const Eigen::Vector3f& attenuation)
    {
        attenuation_ = attenuation;
    }

private:

    LightType light_type_;

    Eigen::Vector3d position_;
    Eigen::Vector3f ambient_;
    Eigen::Vector3f diffuse_;
    Eigen::Vector3f specular_;
    Eigen::Vector3f attenuation_;
};

}


#endif // ITOMP_RENDERER_LIGHT_H