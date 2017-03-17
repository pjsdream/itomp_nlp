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
    LightShadowShader(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename);

    void bindShadowmapTextureDirectional(int light_index, GLuint texture);
    void bindShadowmapTexturePoint(int light_index, GLuint texture);

    virtual void loadLights(const std::vector<Light*>& lights);
    
protected:
    
    virtual void getAllUniformLocations();

private:

    void loadTextureSamplers();

    Camera camera_;

    GLuint location_directional_light_shadow_map_[MAX_NUM_LIGHTS];
    GLuint location_directional_light_projection_view_[MAX_NUM_LIGHTS];

    GLuint location_far_plane_;
    GLuint location_point_light_shadow_map_[MAX_NUM_LIGHTS];
    GLuint location_point_light_projection_view_[MAX_NUM_LIGHTS];
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADOW_SHADER_H