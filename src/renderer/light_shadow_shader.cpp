#include <itomp_nlp/renderer/light_shadow_shader.h>

#include <iostream>


namespace itomp
{

const std::string LightShadowShader::vertex_filename_ = "shader/light_shadow.vert";
const std::string LightShadowShader::fragment_filename_ = "shader/light_shadow.frag";


LightShadowShader::LightShadowShader(Renderer* renderer)
    : LightShader(renderer, vertex_filename_, fragment_filename_)
{
    getAllUniformLocations();

    camera_.setOrtho();
    camera_.setNear(0.0);
    camera_.setFar(3.0);

    start();
    loadTextureSamplers();
    stop();
}

void LightShadowShader::getAllUniformLocations()
{
    location_texture_sampler_ = getUniformLocation("texture_sampler");
    
    for (int i=0; i<MAX_NUM_LIGHTS; i++)
    {
        const std::string array_index = "[" + std::to_string(i) + "]";
        
        location_shadow_map_[i] = getUniformLocation("shadow_map" + array_index);
        location_light_projection_view_[i] = getUniformLocation("light_projection_view" + array_index);
    }
}

void LightShadowShader::loadTextureSamplers()
{
    gl_->glUniform1i(location_texture_sampler_, 0);

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
        gl_->glUniform1i(location_shadow_map_[i], i + 1);
}

void LightShadowShader::bindShadowmapTexture(int light_index, GLuint texture)
{
    gl_->glActiveTexture(GL_TEXTURE1 + light_index);
    gl_->glBindTexture(GL_TEXTURE_2D, texture);
}

void LightShadowShader::loadLights(const std::vector<Light*>& lights)
{
    LightShader::loadLights(lights);

    const float depth = 2.0f;
    for (int i=0; i<lights.size() && i<MAX_NUM_LIGHTS; i++)
    {
        camera_.lookAt(lights[i]->getPosition().normalized() * depth, Eigen::Vector3d::Zero());

        const Eigen::Matrix4f view = camera_.viewMatrix().cast<float>();
        const Eigen::Matrix4f projection = camera_.projectionMatrix().cast<float>();
        const Eigen::Matrix4f projection_view = projection * view;

        loadUniform(location_light_projection_view_[i], projection_view);
    }
}

}
