#ifndef ITOMP_RENDERER_SHADOWMAP_POINT_SHADER_H
#define ITOMP_RENDERER_SHADOWMAP_POINT_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>

#include <itomp_nlp/renderer/camera.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>


namespace itomp
{

class ShadowmapPointShader : public ShaderProgram
{
private:

    static const int MAX_NUM_LIGHTS = 8;

    static const GLuint SHADOW_WIDTH = 512;
    static const GLuint SHADOW_HEIGHT = 512;

    static const std::string vertex_filename_;
    static const std::string geometry_filename_;
    static const std::string fragment_filename_;

public:

    ShadowmapPointShader(Renderer* renderer);
    
    virtual void bindAttributes();

    void bindTexture(int light_index);

    void loadLight(const Light* light);
    
    virtual void loadModelTransform(const Eigen::Matrix4f& m);

    inline GLuint getShadowmapTextureId(int light_index)
    {
        return depth_texture_[light_index];
    }

protected:
    
    virtual void getAllUniformLocations();

private:
    
    Camera camera_;

    GLuint location_model_;
    
    GLuint location_light_position_;
    GLuint location_far_plane_;

    GLuint location_shadow_matrices_[6];

    // framebuffer object
    void initializeFrameBuffer();

    GLuint fbo_[MAX_NUM_LIGHTS];
    GLuint depth_texture_[MAX_NUM_LIGHTS];
};

}


#endif // ITOMP_RENDERER_SHADOWMAP_POINT_SHADER_H