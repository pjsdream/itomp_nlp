#include <itomp_nlp/renderer/shadowmap_point_shader.h>


namespace itomp
{

const std::string ShadowmapPointShader::vertex_filename_ = "shader/shadowmap_point.vert";
const std::string ShadowmapPointShader::geometry_filename_ = "shader/shadowmap_point.geom";
const std::string ShadowmapPointShader::fragment_filename_ = "shader/shadowmap_point.frag";


ShadowmapPointShader::ShadowmapPointShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, geometry_filename_, fragment_filename_)
{
    fbo_[0] = 0;

    bindAttributes();
    getAllUniformLocations();

    camera_.setPerspective();
    camera_.setAspect((GLfloat)SHADOW_WIDTH / (GLfloat)SHADOW_HEIGHT);
    camera_.setFovy(90.);
    camera_.setNear(0.1f);
    camera_.setFar(10.f);
}

void ShadowmapPointShader::initializeFrameBuffer()
{
    gl_->glGenFramebuffers(MAX_NUM_LIGHTS, fbo_);
    gl_->glGenTextures(MAX_NUM_LIGHTS, depth_texture_);

    for (int i=0; i<MAX_NUM_LIGHTS; i++)
    {
        gl_->glBindTexture(GL_TEXTURE_CUBE_MAP, depth_texture_[i]);
        for (int j=0; j<6; j++)
            gl_->glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + j, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

        gl_->glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        gl_->glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gl_->glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl_->glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        gl_->glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        gl_->glBindFramebuffer(GL_FRAMEBUFFER, fbo_[i]);
        gl_->glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_texture_[i], 0);
        gl_->glDrawBuffer(GL_NONE);
        gl_->glReadBuffer(GL_NONE);
        gl_->glBindFramebuffer(GL_FRAMEBUFFER, 0);
        gl_->glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    }
}

void ShadowmapPointShader::bindTexture(int light_index)
{
    if (fbo_[0] == 0)
        initializeFrameBuffer();
    
    gl_->glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    gl_->glBindFramebuffer(GL_FRAMEBUFFER, fbo_[light_index]);
    gl_->glClear(GL_DEPTH_BUFFER_BIT);
}

void ShadowmapPointShader::bindAttributes()
{
    bindAttribute(0, "position");
}

void ShadowmapPointShader::getAllUniformLocations()
{
    location_model_ = getUniformLocation("model");

    location_light_position_ = getUniformLocation("light_position");
    location_far_plane_ = getUniformLocation("far_plane");

    for (int i=0; i<6; i++)
    {
        const std::string array_index = "[" + std::to_string(i) + "]";

        location_shadow_matrices_[i] = getUniformLocation("shadow_matrices" + array_index);
    }
}

void ShadowmapPointShader::loadModelTransform(const Eigen::Matrix4f& m)
{
    loadUniform(location_model_, m);
}

void ShadowmapPointShader::loadLight(const Light* light)
{
    static const Eigen::Vector3d centers[6] = 
    {
        Eigen::Vector3d( 1, 0, 0),
        Eigen::Vector3d(-1, 0, 0),
        Eigen::Vector3d(0,  1, 0),
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0, 0,  1),
        Eigen::Vector3d(0, 0, -1),
    };

    static const Eigen::Vector3d ups[6] = 
    {
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0, 0,  1),
        Eigen::Vector3d(0, 0, -1),
        Eigen::Vector3d(0, -1, 0),
        Eigen::Vector3d(0, -1, 0),
    };
    
    const float far_plane = 10.0f;

    // ASSUMPTION: point light
    const Eigen::Vector3d& light_position = light->getPosition();
    const Eigen::Vector3f light_position_float = light_position.cast<float>();

    loadUniform(location_light_position_, light_position_float);
    loadUniform(location_far_plane_, far_plane);

    for (int i=0; i<6; i++)
    {
        camera_.lookAt(light_position, light_position + centers[i], ups[i]);
        const Eigen::Matrix4f pv = (camera_.projectionMatrix() * camera_.viewMatrix()).cast<float>();

        loadUniform(location_shadow_matrices_[i], pv);
    }
}

}
