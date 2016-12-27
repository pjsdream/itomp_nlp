#include <renderer/static_shader.h>


namespace itomp_renderer
{

const std::string StaticShader::vertex_filename = "shader/static.vert";
const std::string StaticShader::fragment_filename = "shader/static.frag";


StaticShader::StaticShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename, fragment_filename)
{
    bindAttributes();
    getAllUniformLocations();
}

void StaticShader::bindAttributes()
{
    bindAttribute(0, "position");
    bindAttribute(1, "texture_coords");
}

void StaticShader::getAllUniformLocations()
{
    location_transform_matrix_ = getUniformLocation("transformation_matrix");
    location_projection_matrix_ = getUniformLocation("projection_matrix");
}

void StaticShader::loadTransformationMatrix(const Eigen::Matrix4f& matrix)
{
    loadUniform(location_transform_matrix_, matrix);
}

void StaticShader::loadProjectionMatrix(const Eigen::Matrix4f& matrix)
{
    loadUniform(location_projection_matrix_, matrix);
}

}
