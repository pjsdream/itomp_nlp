#include <itomp_nlp/renderer/shadowmap_shader.h>


namespace itomp
{

const std::string ShadowmapShader::vertex_filename_ = "shader/shadowmap.vert";
const std::string ShadowmapShader::fragment_filename_ = "shader/shadowmap.frag";


ShadowmapShader::ShadowmapShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, fragment_filename_)
{
    bindAttributes();
    getAllUniformLocations();
    
    camera_.setOrtho();
    camera_.setNear(0.0);
    camera_.setFar(3.0);
}

void ShadowmapShader::initializeFrameBuffer()
{
    gl_->glGenFramebuffers(1, &fbo_);

    gl_->glGenTextures(1, &depth_texture_);
    gl_->glBindTexture(GL_TEXTURE_2D, depth_texture_);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    GLfloat border_color[] = { 1.0, 1.0, 1.0, 1.0 };
    gl_->glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);

    gl_->glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    gl_->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);
    gl_->glDrawBuffer(GL_NONE);
    gl_->glReadBuffer(GL_NONE);
    gl_->glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ShadowmapShader::start()
{
    ShaderProgram::start();
    
    if (fbo_ == 0)
        initializeFrameBuffer();
    
    gl_->glGetIntegerv(GL_FRAMEBUFFER_BINDING, &screen_fbo_);

    gl_->glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    gl_->glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    gl_->glClear(GL_DEPTH_BUFFER_BIT);
}

void ShadowmapShader::stop()
{
    ShaderProgram::stop();

    gl_->glBindFramebuffer(GL_FRAMEBUFFER, screen_fbo_);

    // bind texture
    gl_->glBindTexture(GL_TEXTURE_2D, depth_texture_);
}

void ShadowmapShader::bindAttributes()
{
    bindAttribute(0, "position");
}

void ShadowmapShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");
    location_view_ = getUniformLocation("view");
    location_projection_ = getUniformLocation("projection");
}

void ShadowmapShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void ShadowmapShader::loadLight(const Light* light)
{
    // ASSUMPTION: directional light
    const float depth = 2.0f;
    camera_.lookAt(light->getPosition().normalized() * depth, Eigen::Vector3d::Zero());

    const Eigen::Matrix4f view = camera_.viewMatrix().cast<float>();
    const Eigen::Matrix4f projection = camera_.projectionMatrix().cast<float>();

    loadUniform(location_view_, view);
    loadUniform(location_projection_, projection);
}

}
