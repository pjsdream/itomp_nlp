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

    void bindOpaqueTextures(GLuint color_texture, GLuint depth_texture);

protected:
    
private:

    virtual void bindAttributes();
    virtual void getAllUniformLocations();

    void initializeQuadBuffer();

    // quad
    GLuint vao_;
    GLuint vbo_;

    // textures
    GLuint location_color_texture_;
    GLuint location_depth_texture_;
};

}


#endif // ITOMP_RENDERER_OIT_RESOLVE_SHADER_H