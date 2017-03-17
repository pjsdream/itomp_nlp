#ifndef ITOMP_RENDERER_OIT_RESOLVE_SHADER_H
#define ITOMP_RENDERER_OIT_RESOLVE_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>


namespace itomp
{

class OITResolveShader : public ShaderProgram
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    OITResolveShader(Renderer* renderer);
    ~OITResolveShader();

    void resolve();

protected:
    
private:

    virtual void bindAttributes();

    void initializeQuadBuffer();

    GLuint vao_;
    GLuint vbo_;
};

}


#endif // ITOMP_RENDERER_OIT_RESOLVE_SHADER_H