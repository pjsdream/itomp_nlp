#include <renderer/light_shader.h>


namespace itomp_renderer
{

const std::string LightShader::vertex_filename_ = "shader/light.vert";
const std::string LightShader::fragment_filename_ = "shader/light.frag";


LightShader::LightShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, fragment_filename_)
{
    bindAttributes();
    getAllUniformLocations();
}

void LightShader::bindAttributes()
{
    bindAttribute(0, "position");
    bindAttribute(1, "normal");
    bindAttribute(2, "texture_coords");
}

void LightShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");
    location_view_ = getUniformLocation("view");
    location_projection_ = getUniformLocation("projection");

    location_light_position_ = getUniformLocation("light_position");
    location_light_ambient_color_ = getUniformLocation("light_ambient_color");
    location_light_diffuse_color_ = getUniformLocation("light_diffuse_color");
    location_light_specular_color_ = getUniformLocation("light_specular_color");
    location_light_ambient_ = getUniformLocation("light_ambient");
    location_eye_position_ = getUniformLocation("eye_position");

    location_material_ambient_color_ = getUniformLocation("material_ambient_color");
    location_material_specular_color_ = getUniformLocation("material_specular_color");
}

void LightShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void LightShader::loadCamera(const Camera& camera)
{
    const Eigen::Matrix4f view = camera.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera.projectionMatrix().cast<float>();
    const Eigen::Vector3f eye_position = camera.eyePosition().cast<float>();

    loadUniform(location_view_, view);
    loadUniform(location_projection_, projection);
    loadUniform(location_eye_position_, eye_position);
}

void LightShader::loadLight(const Light* light)
{
    const Eigen::Vector3f position = light->getPosition().cast<float>();
    const Eigen::Vector3f ambient_color = light->getAmbientColor().cast<float>();
    const Eigen::Vector3f diffuse_color = light->getDiffuseColor().cast<float>();
    const Eigen::Vector3f specular_color = light->getSpecularColor().cast<float>();

    loadUniform(location_light_position_, position);
    loadUniform(location_light_ambient_color_, ambient_color);
    loadUniform(location_light_diffuse_color_, diffuse_color);
    loadUniform(location_light_specular_color_, specular_color);
}

void LightShader::loadMaterial(const Material* material)
{
    const Eigen::Vector3f ambient_color = material->getAmbientColor().cast<float>();
    const Eigen::Vector3f specular_color = material->getSpecularColor().cast<float>();

    loadUniform(location_material_ambient_color_, ambient_color);
    loadUniform(location_material_specular_color_, specular_color);
}

}
