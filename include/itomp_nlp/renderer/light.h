#ifndef ITOMP_RENDERER_LIGHT_H
#define ITOMP_RENDERER_LIGHT_H


#include <Eigen/Dense>


namespace itomp_renderer
{

class Light
{
public:

    Light(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(), const Eigen::Vector3d& color = Eigen::Vector3d(1, 1, 1));

    inline const Eigen::Vector3d& getPosition() const
    {
        return position_;
    }

    inline Eigen::Vector3d getAmbientColor() const
    {
        return Eigen::Vector3d::Zero();
    }

    inline const Eigen::Vector3d& getDiffuseColor() const
    {
        return color_;
    }

    inline const Eigen::Vector3d& getSpecularColor() const
    {
        return color_;
    }

    inline void setPosition(const Eigen::Vector3d& position)
    {
        position_ = position;
    }

    inline void setColor(const Eigen::Vector3d& color)
    {
        color_ = color;
    }

private:

    Eigen::Vector3d position_;
    Eigen::Vector3d color_;
};

}


#endif // ITOMP_RENDERER_LIGHT_H