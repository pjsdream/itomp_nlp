#include <itomp_nlp/renderer/color_shader.h>


namespace itomp
{

const std::string ColorShader::vertex_filename_ = "shader/color.vert";
const std::string ColorShader::fragment_filename_ = "shader/color.frag";


ColorShader::ColorShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, fragment_filename_)
{
    bindAttributes();
    getAllUniformLocations();
}

void ColorShader::bindAttributes()
{
    bindAttribute(0, "position");
    bindAttribute(1, "color");
}

void ColorShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");
    location_view_ = getUniformLocation("view");
    location_projection_ = getUniformLocation("projection");
}

void ColorShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void ColorShader::loadCamera(const Camera& camera)
{
    const Eigen::Matrix4f view = camera.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera.projectionMatrix().cast<float>();
    const Eigen::Vector3f eye_position = camera.eyePosition().cast<float>();

    loadUniform(location_view_, view);
    loadUniform(location_projection_, projection);
}

}
