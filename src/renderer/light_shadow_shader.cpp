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
    location_far_plane_ = getUniformLocation("far_plane");

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
    {
        const std::string array_index = "[" + std::to_string(i) + "]";
        
        location_directional_light_projection_view_[i] = getUniformLocation("directional_lights" + array_index + ".projection_view");
        location_directional_light_shadow_map_[i] = getUniformLocation("directional_lights" + array_index + ".shadow_map");
        
        location_point_light_shadow_map_[i] = getUniformLocation("point_lights" + array_index + ".shadow_map");
    }
}

void LightShadowShader::loadTextureSamplers()
{
    gl_->glUniform1i(location_material_diffuse_texture_, 0);

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
        gl_->glUniform1i(location_directional_light_shadow_map_[i], i + 1);

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
        gl_->glUniform1i(location_point_light_shadow_map_[i], i + MAX_NUM_LIGHTS + 1);
}

void LightShadowShader::bindShadowmapTextureDirectional(int light_index, GLuint texture)
{
    gl_->glActiveTexture(GL_TEXTURE1 + light_index);
    gl_->glBindTexture(GL_TEXTURE_2D, texture);
}

void LightShadowShader::bindShadowmapTexturePoint(int light_index, GLuint texture)
{
    gl_->glActiveTexture(GL_TEXTURE1 + MAX_NUM_LIGHTS + light_index);
    gl_->glBindTexture(GL_TEXTURE_CUBE_MAP, texture);
}

void LightShadowShader::loadLights(const std::vector<Light*>& lights)
{
    LightShader::loadLights(lights);

    int didx = 0;

    const float depth = 2.0f;
    for (int i=0; i<lights.size() && i<MAX_NUM_LIGHTS; i++)
    {
        if (lights[i]->isDirectional())
        {
            camera_.lookAt(lights[i]->getPosition().normalized() * depth, Eigen::Vector3d::Zero());

            const Eigen::Matrix4f view = camera_.viewMatrix().cast<float>();
            const Eigen::Matrix4f projection = camera_.projectionMatrix().cast<float>();
            const Eigen::Matrix4f projection_view = projection * view;

            loadUniform(location_directional_light_projection_view_[didx], projection_view);
            didx++;
        }
    }

    loadUniform(location_far_plane_, 10.0f);
}

}
