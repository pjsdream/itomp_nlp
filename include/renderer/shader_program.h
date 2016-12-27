#ifndef ITOMP_RENDERER_SHADER_PROGRAM_H
#define ITOMP_RENDERER_SHADER_PROGRAM_H


#include <renderer/gl_base.h>


namespace itomp_renderer
{

class ShaderProgram : public GLBase
{
public:

    ShaderProgram(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename);

    virtual void bindAttributes() = 0;

    void start();
    void stop();
    void cleanUp();

protected:

    void bindAttribute(int attribute, const std::string& variable_name);

private:

    GLuint loadShader(const std::string& filename, GLuint shader_type);
    GLuint createShaderProgram();

    GLuint vertex_shader_;
    GLuint fragment_shader_;

    GLuint shader_program_;
};

}


#endif // ITOMP_RENDERER_SHADER_PROGRAM_H