#ifndef ITOMP_RENDERER_SHAPE_H
#define ITOMP_RENDERER_SHAPE_H


#include <itomp_nlp/shape/shape.h>

#include <itomp_nlp/renderer/gl_base.h>
#include <itomp_nlp/renderer/light_shader.h>

#include <Eigen/Dense>


namespace itomp
{

class RenderingShape : public GLBase
{
public:

    RenderingShape(Renderer* renderer);
    
    virtual void draw(LightShader* shader) = 0;

protected:

};

}


#endif // ITOMP_RENDERER_SHAPE_H