#include <renderer/static_shader.h>


namespace itomp_renderer
{

const std::string StaticShader::vertex_filename = "shader/static.vert";
const std::string StaticShader::fragment_filename = "shader/static.frag";


StaticShader::StaticShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename, fragment_filename)
{
    bindAttributes();
}

void StaticShader::bindAttributes()
{
    bindAttribute(0, "position");
    bindAttribute(1, "texture_coords");
}

}
