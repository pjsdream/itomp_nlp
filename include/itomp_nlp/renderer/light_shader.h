#ifndef ITOMP_RENDERER_LIGHT_SHADER_H
#define ITOMP_RENDERER_LIGHT_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>

#include <itomp_nlp/renderer/camera.h>
#include <itomp_nlp/renderer/light.h>


namespace itomp
{

class LightShader : public ShaderProgram
{
protected:

    static const int MAX_NUM_LIGHTS = 8;

private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    LightShader(Renderer* renderer);
    LightShader(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename);
    
    virtual void loadModelTransform(const Eigen::Matrix4f& m);
    virtual void loadMaterial(const Material* material);

    virtual void loadLights(const std::vector<Light*>& lights);

    void loadCamera(const Camera& camera);

protected:
    
    virtual void bindAttributes();
    virtual void getAllUniformLocations();

    GLuint location_model_;
    GLuint location_view_;
    GLuint location_projection_;

    GLuint location_directional_light_use_[MAX_NUM_LIGHTS];
    GLuint location_directional_light_position_[MAX_NUM_LIGHTS];
    GLuint location_directional_light_ambient_[MAX_NUM_LIGHTS];
    GLuint location_directional_light_diffuse_[MAX_NUM_LIGHTS];
    GLuint location_directional_light_specular_[MAX_NUM_LIGHTS];

    GLuint location_point_light_use_[MAX_NUM_LIGHTS];
    GLuint location_point_light_position_[MAX_NUM_LIGHTS];
    GLuint location_point_light_ambient_[MAX_NUM_LIGHTS];
    GLuint location_point_light_diffuse_[MAX_NUM_LIGHTS];
    GLuint location_point_light_specular_[MAX_NUM_LIGHTS];
    GLuint location_point_light_attenuation_[MAX_NUM_LIGHTS];

    GLuint location_eye_position_;
    
    GLuint location_material_ambient_;
    GLuint location_material_diffuse_;
    GLuint location_material_specular_;
    GLuint location_material_shininess_;
    GLuint location_material_has_texture_;

    GLuint location_material_diffuse_texture_;
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADER_H