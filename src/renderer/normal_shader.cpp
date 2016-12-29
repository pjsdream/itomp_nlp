#include <itomp_nlp/renderer/normal_shader.h>


namespace itomp_renderer
{

const std::string NormalShader::vertex_filename_ = "shader/normal.vert";
const std::string NormalShader::geometry_filename_ = "shader/normal.geom";
const std::string NormalShader::fragment_filename_ = "shader/normal.frag";


NormalShader::NormalShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, geometry_filename_, fragment_filename_)
{
    bindAttributes();
    getAllUniformLocations();
}

void NormalShader::bindAttributes()
{
    bindAttribute(0, "position");
    bindAttribute(1, "normal");
}

void NormalShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");
    location_view_ = getUniformLocation("view");
    location_projection_ = getUniformLocation("projection");

    location_line_length_ = getUniformLocation("line_length");
}

void NormalShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void NormalShader::loadCamera(const Camera& camera)
{
    const Eigen::Matrix4f view = camera.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera.projectionMatrix().cast<float>();

    loadUniform(location_view_, view);
    loadUniform(location_projection_, projection);
}

void NormalShader::loadLineLength(float length)
{
    loadUniform(location_line_length_, length);
}

}
