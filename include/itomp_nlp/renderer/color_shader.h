#ifndef ITOMP_RENDERER_COLOR_SHADER_H
#define ITOMP_RENDERER_COLOR_SHADER_H


#include <itomp_nlp/renderer/shader_program.h>

#include <itomp_nlp/renderer/camera.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>


namespace itomp
{

class ColorShader : public ShaderProgram
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    ColorShader(Renderer* renderer);
    
    virtual void bindAttributes();

    void loadModelTransform(const Eigen::Matrix4f& m);
    void loadCamera(const Camera& camera);

protected:
    
    virtual void getAllUniformLocations();

private:
    
    GLuint location_model_;
    GLuint location_view_;
    GLuint location_projection_;
};

}


#endif // ITOMP_RENDERER_COLOR_SHADER_H