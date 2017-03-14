#ifndef ITOMP_RENDERER_SHADOWMAP_SHADER_H
#define ITOMP_RENDERER_SHADOWMAP_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>

#include <itomp_nlp/renderer/camera.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>


namespace itomp
{

class ShadowmapShader : public ShaderProgram
{
private:

    static const GLuint SHADOW_WIDTH = 1024;
    static const GLuint SHADOW_HEIGHT = 1024;

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    ShadowmapShader(Renderer* renderer);
    
    virtual void bindAttributes();

    virtual void start();
    virtual void stop();

    void loadModelTransform(const Eigen::Matrix4f& m);
    void loadLight(const Light* light);

    inline GLuint getShadowmapTextureId()
    {
        return depth_texture_;
    }

protected:
    
    virtual void getAllUniformLocations();

private:
    
    Camera camera_;

    GLuint location_model_;
    GLuint location_view_;
    GLuint location_projection_;

    // framebuffer object
    void initializeFrameBuffer();

    GLint screen_fbo_;
    GLuint fbo_;
    GLuint depth_texture_;
};

}


#endif // ITOMP_RENDERER_SHADOWMAP_SHADER_H