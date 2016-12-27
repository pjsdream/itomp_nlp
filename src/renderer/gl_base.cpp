#include <renderer/gl_base.h>

#include <renderer/renderer.h>


namespace itomp_renderer
{

GLBase::GLBase(Renderer* renderer)
    : gl_(renderer->getGLFunctions())
{
}

GLBase::GLBase(GLBase& base)
    : gl_(base.gl_)
{
}

GLBase::GLBase(QOpenGLFunctions_4_3_Core* gl)
    : gl_(gl)
{
}

}
