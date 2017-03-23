#ifndef ITOMP_RENDERER_PLANE_H
#define ITOMP_RENDERER_PLANE_H


#include <itomp_nlp/renderer/rendering_shape.h>


namespace itomp
{

// xy plane transformed by transform_ variable
class RenderingPlane : public RenderingShape
{
public:

    RenderingPlane(Renderer* renderer);
    ~RenderingPlane();

    virtual void draw(ShaderProgram* shader);
    
private:
    
    bool need_update_buffer_;

    void updateBuffers();

    GLuint vao_;
    std::vector<GLuint> vbos_;
};

}


#endif // ITOMP_RENDERER_BOX_H