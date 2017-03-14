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
}

void LightShadowShader::getAllUniformLocations()
{
    location_texture_sampler_ = getUniformLocation("texture_sampler");
    location_shadow_map_ = getUniformLocation("shadow_map");
    
    location_light_projection_view_ = getUniformLocation("light_projection_view");
}

void LightShadowShader::bindShadowmapTexture(GLuint texture)
{
    gl_->glUniform1i(location_texture_sampler_, 0);
    gl_->glUniform1i(location_shadow_map_, 1);

    gl_->glActiveTexture(GL_TEXTURE1);
    gl_->glBindTexture(GL_TEXTURE_2D, texture);
}

void LightShadowShader::loadLights(const std::vector<Light*>& lights)
{
    LightShader::loadLights(lights);

    const float depth = 2.0f;
    camera_.lookAt(lights[0]->getPosition().normalized() * depth, Eigen::Vector3d::Zero());

    const Eigen::Matrix4f view = camera_.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera_.projectionMatrix().cast<float>();
    const Eigen::Matrix4f projection_view = projection * view;

    loadUniform(location_light_projection_view_, projection_view);
}

}
