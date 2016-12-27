#ifndef ITOMP_RENDERER_GL_BASE_H
#define ITOMP_RENDERER_GL_BASE_H


#include <QOpenGLFunctions_4_3_Core>


namespace itomp_renderer
{

class Renderer;

class GLBase
{
public:

    GLBase(QOpenGLFunctions_4_3_Core* gl);
    GLBase(Renderer* renderer);
    GLBase(GLBase& base);

protected:

    QOpenGLFunctions_4_3_Core* gl_;
};

}


#endif // ITOMP_RENDERER_GL_BASE_H