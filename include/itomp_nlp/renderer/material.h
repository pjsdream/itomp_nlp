#ifndef ITOMP_RENDERER_MATERIAL_H
#define ITOMP_RENDERER_MATERIAL_H


#include <Eigen/Dense>


namespace itomp_renderer
{

class Material
{
public:

    Material(const Eigen::Vector3d& ambient_color = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d& specular_color = Eigen::Vector3d(0, 0, 0));

    inline const Eigen::Vector3d& getAmbientColor() const
    {
        return ambient_color_;
    }

    inline const Eigen::Vector3d& getDiffuseColor() const
    {
        return diffuse_color_;
    }

    inline const Eigen::Vector3d& getSpecularColor() const
    {
        return specular_color_;
    }

private:
    
    Eigen::Vector3d ambient_color_;
    Eigen::Vector3d diffuse_color_;
    Eigen::Vector3d specular_color_;
};

}


#endif // ITOMP_RENDERER_MATERIAL_H