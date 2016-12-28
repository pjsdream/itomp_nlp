#include <itomp_nlp/renderer/gl_base.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp_renderer
{

GLBase::GLBase(Renderer* renderer)
    : renderer_(renderer)
    , gl_(renderer->getGLFunctions())
{
}

}
