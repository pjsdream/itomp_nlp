#ifndef ITOMP_RENDERER_STATIC_SHADER_H
#define ITOMP_RENDERER_STATIC_SHADER_H


#include <renderer/shader_program.h>


namespace itomp_renderer
{

class StaticShader : public ShaderProgram
{
private:

    static const std::string vertex_filename;
    static const std::string fragment_filename;

public:

    StaticShader(Renderer* renderer);

    virtual void bindAttributes();

private:
};

}


#endif // ITOMP_RENDERER_STATIC_SHADER_H