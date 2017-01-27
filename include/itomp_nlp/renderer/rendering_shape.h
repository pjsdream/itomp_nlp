#ifndef ITOMP_RENDERER_SHAPE_H
#define ITOMP_RENDERER_SHAPE_H


#include <itomp_nlp/renderer/gl_base.h>
#include <itomp_nlp/renderer/material.h>

#include <Eigen/Dense>


namespace itomp
{

class RenderingShape : public GLBase
{
public:

    RenderingShape(Renderer* renderer);
    
    virtual void draw(GLuint primitive_type = GL_TRIANGLES);

protected:
    
    GLuint vao_;
    std::vector<GLuint> vbos_;
    int num_vertices_;

    Material* material_;
};

}


#endif // ITOMP_RENDERER_SHAPE_H