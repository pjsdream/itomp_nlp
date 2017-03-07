#ifndef ITOMP_RENDERER_SHAPE_H
#define ITOMP_RENDERER_SHAPE_H


#include <itomp_nlp/renderer/renderer.h>

#include <itomp_nlp/shape/shape.h>

#include <itomp_nlp/renderer/gl_base.h>
#include <itomp_nlp/renderer/light_shader.h>
#include <itomp_nlp/renderer/color_shader.h>

#include <itomp_nlp/renderer/material.h>

#include <Eigen/Dense>


namespace itomp
{

class RenderingShape : public GLBase
{
public:

    RenderingShape(Renderer* renderer, Renderer::ShaderType shader = Renderer::SHADER_TYPE_LIGHT);
    ~RenderingShape();
    
    virtual void draw(LightShader* shader);
    virtual void draw(ColorShader* shader);

    void setMaterial(Material* material);

protected:

    Material* material_;
};

}


#endif // ITOMP_RENDERER_SHAPE_H