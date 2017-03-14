#ifndef ITOMP_RENDERER_LIGHT_SHADER_H
#define ITOMP_RENDERER_LIGHT_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>

#include <itomp_nlp/renderer/camera.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>


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
    
    virtual void bindAttributes();

    void loadModelTransform(const Eigen::Matrix4f& m);
    void loadCamera(const Camera& camera);

    virtual void loadLights(const std::vector<Light*>& lights);

    void loadMaterial(const Material* material);

protected:
    
    virtual void getAllUniformLocations();

private:
    
    GLuint location_model_;
    GLuint location_view_;
    GLuint location_projection_;

    GLuint location_light_use_[MAX_NUM_LIGHTS];
    GLuint location_light_position_[MAX_NUM_LIGHTS];
    GLuint location_light_diffuse_color_[MAX_NUM_LIGHTS];
    GLuint location_light_specular_color_[MAX_NUM_LIGHTS];
    GLuint location_eye_position_;

    GLuint location_material_diffuse_color_;
    GLuint location_material_specular_color_;
    GLuint location_shininess_;
    GLuint location_has_texture_;
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADER_H