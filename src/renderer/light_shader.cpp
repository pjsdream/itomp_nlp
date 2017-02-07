#include <itomp_nlp/renderer/light_shader.h>


namespace itomp
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

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
    {
        const std::string array_index = "[" + std::to_string(i) + "]";
        
        location_light_use_[i] = getUniformLocation("light_use" + array_index);
        location_light_position_[i] = getUniformLocation("light_position" + array_index);
        location_light_diffuse_color_[i] = getUniformLocation("light_diffuse_color" + array_index);
        location_light_specular_color_[i] = getUniformLocation("light_specular_color" + array_index);
    }
    location_eye_position_ = getUniformLocation("eye_position");

    location_material_diffuse_color_ = getUniformLocation("material_diffuse_color");
    location_material_specular_color_ = getUniformLocation("material_specular_color");
    location_shininess_ = getUniformLocation("shininess");
    location_has_texture_ = getUniformLocation("has_texture");
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

void LightShader::loadLights(const std::vector<Light*>& lights)
{
    for (int i=0; i<lights.size() && i<MAX_NUM_LIGHTS; i++)
    {
        const Eigen::Vector3f position = lights[i]->getPosition().cast<float>();
        const Eigen::Vector4f diffuse_color = lights[i]->getDiffuseColor();
        const Eigen::Vector4f specular_color = lights[i]->getSpecularColor();
        
        loadUniform(location_light_use_[i], true);
        loadUniform(location_light_position_[i], position);
        loadUniform(location_light_diffuse_color_[i], diffuse_color);
        loadUniform(location_light_specular_color_[i], specular_color);
    }

    for (int i=lights.size(); i<MAX_NUM_LIGHTS; i++)
    {
        loadUniform(location_light_use_[i], false);
    }
}

void LightShader::loadMaterial(const Material* material)
{
    if (material == 0)
    {
    }

    else
    {
        const Eigen::Vector4f specular_color = material->getSpecularColor();

        loadUniform(location_material_specular_color_, specular_color);
        loadUniform(location_shininess_, material->getShininess());

        if (material->hasDiffuseTexture())
        {
            gl_->glActiveTexture(GL_TEXTURE0);
            gl_->glBindTexture(GL_TEXTURE_2D, material->getDiffuseTexture()->getTexture());

            loadUniform(location_has_texture_, true);
        }
        else
        {
            loadUniform(location_has_texture_, false);

            const Eigen::Vector4f diffuse_color = material->getDiffuseColor();

            loadUniform(location_material_diffuse_color_, diffuse_color);
        }
    }
}

}
