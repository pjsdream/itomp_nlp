#ifndef ITOMP_RENDERER_STATIC_SHADER_H
#define ITOMP_RENDERER_STATIC_SHADER_H


#include <renderer/shader_program.h>


namespace itomp_renderer
{

class StaticShader : public ShaderProgram
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

public:

    StaticShader(Renderer* renderer);

    virtual void bindAttributes();

    void loadTransformationMatrix(const Eigen::Matrix4f& matrix);
    void loadProjectionMatrix(const Eigen::Matrix4f& matrix);

protected:
    
    virtual void getAllUniformLocations();

private:

    GLint location_transform_matrix_;
    GLint location_projection_matrix_;
};

}


#endif // ITOMP_RENDERER_STATIC_SHADER_H