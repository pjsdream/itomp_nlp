#ifndef ITOMP_RENDERER_LIGHT_SHADER_H
#define ITOMP_RENDERER_LIGHT_SHADER_H


#include <renderer/shader_program.h>

#include <renderer/camera.h>
#include <renderer/light.h>
#include <renderer/material.h>


namespace itomp_renderer
{

class LightShader : public ShaderProgram
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    LightShader(Renderer* renderer);

    virtual void bindAttributes();

    void loadModelTransform(const Eigen::Matrix4f& m);
    void loadCamera(const Camera& camera);

    void loadLight(const Light* light);

    void loadMaterial(const Material* material);

protected:
    
    virtual void getAllUniformLocations();

private:
    
    GLuint location_model_;
    GLuint location_view_;
    GLuint location_projection_;

    GLuint location_light_position_;
    GLuint location_light_ambient_color_;
    GLuint location_light_diffuse_color_;
    GLuint location_light_specular_color_;
    GLuint location_light_ambient_;
    GLuint location_eye_position_;

    GLuint location_material_ambient_color_;
    GLuint location_material_specular_color_;
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADER_H