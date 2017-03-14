#ifndef ITOMP_RENDERER_SHAPE_H
#define ITOMP_RENDERER_SHAPE_H


#include <itomp_nlp/renderer/renderer.h>

#include <itomp_nlp/shape/shape.h>

#include <itomp_nlp/renderer/gl_base.h>
#include <itomp_nlp/renderer/light_shader.h>
#include <itomp_nlp/renderer/color_shader.h>
#include <itomp_nlp/renderer/shadowmap_shader.h>

#include <itomp_nlp/renderer/material.h>

#include <Eigen/Dense>


namespace itomp
{

class RenderingShape : public GLBase
{
public:

    RenderingShape(Renderer* renderer);
    ~RenderingShape();
    
    virtual void draw(LightShader* shader);
    virtual void draw(ColorShader* shader);
    virtual void draw(ShadowmapShader* shader);

    inline void setMaterial(Material* material)
    {
        material_ = material;
    }

    inline const Material* getMaterial() const
    {
        return material_;
    }
    
    void setTransform(const Eigen::Matrix4f& transform);
    void setTransform(const Eigen::Affine3d& transform);

    inline const Eigen::Matrix4f& getTransform() const
    {
        return transform_;
    }

protected:

    Material* material_;

    Eigen::Matrix4f transform_;
};

}


#endif // ITOMP_RENDERER_SHAPE_H