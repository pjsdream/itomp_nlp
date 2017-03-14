#ifndef ITOMP_RENDERER_LIGHT_SHADOW_SHADER_H
#define ITOMP_RENDERER_LIGHT_SHADOW_SHADER_H


#include <itomp_nlp/renderer/light_shader.h>


namespace itomp
{

class LightShadowShader : public LightShader
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    LightShadowShader(Renderer* renderer);

    void bindShadowmapTexture(GLuint texture);

    virtual void loadLights(const std::vector<Light*>& lights);
    
protected:
    
    virtual void getAllUniformLocations();

private:
    
    Camera camera_;

    GLuint location_texture_sampler_;
    GLuint location_shadow_map_;

    GLuint location_light_projection_view_;
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADOW_SHADER_H