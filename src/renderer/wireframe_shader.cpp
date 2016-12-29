#include <itomp_nlp/renderer/wireframe_shader.h>


namespace itomp_renderer
{

const std::string WireframeShader::vertex_filename_ = "shader/wireframe.vert";
const std::string WireframeShader::geometry_filename_ = "shader/wireframe.geom";
const std::string WireframeShader::fragment_filename_ = "shader/wireframe.frag";


WireframeShader::WireframeShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, geometry_filename_, fragment_filename_)
{
    bindAttributes();
    getAllUniformLocations();
}

void WireframeShader::bindAttributes()
{
    bindAttribute(0, "position");
}

void WireframeShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");
    location_view_ = getUniformLocation("view");
    location_projection_ = getUniformLocation("projection");
}

void WireframeShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void WireframeShader::loadCamera(const Camera& camera)
{
    const Eigen::Matrix4f view = camera.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera.projectionMatrix().cast<float>();

    loadUniform(location_view_, view);
    loadUniform(location_projection_, projection);
}

}
