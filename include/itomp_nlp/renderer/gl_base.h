#ifndef ITOMP_RENDERER_GL_BASE_H
#define ITOMP_RENDERER_GL_BASE_H


#include <QOpenGLFunctions_4_3_Core>


namespace itomp_renderer
{

class Renderer;

class GLBase
{
public:

    GLBase(Renderer* renderer);

    inline Renderer* getRenderer()
    {
        return renderer_;
    }

protected:

    Renderer* renderer_;
    QOpenGLFunctions_4_3_Core* gl_;
};

}


#endif // ITOMP_RENDERER_GL_BASE_H