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

LightShader::LightShader(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename)
    : ShaderProgram(renderer, vertex_filename, fragment_filename)
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
        
        location_directional_light_use_[i] = getUniformLocation("directional_lights" + array_index + ".use");
        location_directional_light_position_[i] = getUniformLocation("directional_lights" + array_index + ".position");
        location_directional_light_ambient_[i] = getUniformLocation("directional_lights" + array_index + ".ambient");
        location_directional_light_diffuse_[i] = getUniformLocation("directional_lights" + array_index + ".diffuse");
        location_directional_light_specular_[i] = getUniformLocation("directional_lights" + array_index + ".specular");

        location_point_light_use_[i] = getUniformLocation("point_lights" + array_index + ".use");
        location_point_light_position_[i] = getUniformLocation("point_lights" + array_index + ".position");
        location_point_light_ambient_[i] = getUniformLocation("point_lights" + array_index + ".ambient");
        location_point_light_diffuse_[i] = getUniformLocation("point_lights" + array_index + ".diffuse");
        location_point_light_specular_[i] = getUniformLocation("point_lights" + array_index + ".specular");
        location_point_light_attenuation_[i] = getUniformLocation("point_lights" + array_index + ".attenuation");
    }
    location_eye_position_ = getUniformLocation("eye_position");
    
    location_material_alpha_ = getUniformLocation("material.alpha");
    location_material_ambient_ = getUniformLocation("material.ambient");
    location_material_diffuse_ = getUniformLocation("material.diffuse");
    location_material_specular_ = getUniformLocation("material.specular");
    location_material_shininess_ = getUniformLocation("material.shininess");
    location_material_has_texture_ = getUniformLocation("material.has_texture");
    location_material_diffuse_texture_= getUniformLocation("material.diffuse_texture");
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
    for (int i=0; i<MAX_NUM_LIGHTS; i++)
    {
        loadUniform(location_directional_light_use_[i], false);
        loadUniform(location_point_light_use_[i], false);
    }

    int didx = 0;
    int pidx = 0;

    for (int i=0; i<lights.size(); i++)
    {
        if (lights[i]->isDirectional() && didx < MAX_NUM_LIGHTS)
        {
            const Eigen::Vector3f& position = lights[i]->getPosition().cast<float>();
            const Eigen::Vector3f& ambient = lights[i]->getAmbient();
            const Eigen::Vector3f& diffuse = lights[i]->getDiffuse();
            const Eigen::Vector3f& specular = lights[i]->getSpecular();
        
            loadUniform(location_directional_light_use_[didx], true);
            loadUniform(location_directional_light_position_[didx], position);
            loadUniform(location_directional_light_ambient_[didx], ambient);
            loadUniform(location_directional_light_diffuse_[didx], diffuse);
            loadUniform(location_directional_light_specular_[didx], specular);
            didx++;
        }
        else if (lights[i]->isPoint() && pidx < MAX_NUM_LIGHTS)
        {
            const Eigen::Vector3f& position = lights[i]->getPosition().cast<float>();
            const Eigen::Vector3f& ambient = lights[i]->getAmbient();
            const Eigen::Vector3f& diffuse = lights[i]->getDiffuse();
            const Eigen::Vector3f& specular = lights[i]->getSpecular();
            const Eigen::Vector3f& attenuation = lights[i]->getAttenuation();
        
            loadUniform(location_point_light_use_[pidx], true);
            loadUniform(location_point_light_position_[pidx], position);
            loadUniform(location_point_light_ambient_[pidx], ambient);
            loadUniform(location_point_light_diffuse_[pidx], diffuse);
            loadUniform(location_point_light_specular_[pidx], specular);
            loadUniform(location_point_light_attenuation_[pidx], attenuation);

            pidx++;
        }
    }
}

void LightShader::loadMaterial(const Material* material)
{
    if (material == 0)
    {
        loadUniform(location_material_has_texture_, false);
        loadUniform(location_material_alpha_, 1.0f);

        printf("material = 0\n");
    }

    else
    {
        const Eigen::Vector3f& ambient = material->getAmbient();
        const Eigen::Vector3f& specular = material->getSpecular();
        
        loadUniform(location_material_alpha_, material->getAlpha());
        loadUniform(location_material_ambient_, ambient);
        loadUniform(location_material_specular_, specular);
        loadUniform(location_material_shininess_, material->getShininess());
        
        if (material->hasDiffuseTexture())
        {
            gl_->glActiveTexture(GL_TEXTURE2);
            gl_->glUniform1i(location_material_diffuse_texture_, 2);
            gl_->glBindTexture(GL_TEXTURE_2D, material->getDiffuseTexture()->getTexture());
            gl_->glActiveTexture(GL_TEXTURE0);

            loadUniform(location_material_has_texture_, true);
        }
        else
        {
            loadUniform(location_material_has_texture_, false);

            const Eigen::Vector3f& diffuse = material->getDiffuse();

            loadUniform(location_material_diffuse_, diffuse);
        }
    }
}

}
