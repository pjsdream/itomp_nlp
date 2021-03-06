#ifndef ITOMP_RENDERER_SHADER_PROGRAM_H
#define ITOMP_RENDERER_SHADER_PROGRAM_H


#include <itomp_nlp/renderer/gl_base.h>

#include <itomp_nlp/renderer/material.h>

#include <Eigen/Dense>


namespace itomp
{

class ShaderProgram : public GLBase
{
public:

    ShaderProgram(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename);
    ShaderProgram(Renderer* renderer, const std::string& vertex_filename, const std::string& geometry_filename, const std::string& fragment_filename);
    ~ShaderProgram();

    virtual void start();
    void stop();
    
    void loadUniform(int location, int value);
    void loadUniform(int location, float value);
    void loadUniform(int locatiom, const Eigen::Vector3f& v);
    void loadUniform(int location, const Eigen::Vector4f& v);
    void loadUniform(int location, bool value);
    void loadUniform(int location, const Eigen::Matrix4f& m);

    virtual void loadModelTransform(const Eigen::Matrix4f& m) {}
    virtual void loadMaterial(const Material* material) {}

protected:
    
    virtual void bindAttributes() = 0;
    virtual void getAllUniformLocations();

    void bindAttribute(int attribute, const std::string& variable_name);

    GLint getUniformLocation(const std::string& uniform_name);

private:

    GLuint loadShader(const std::string& filename, GLuint shader_type);
    GLuint createShaderProgram();

    GLuint vertex_shader_;
    GLuint geometry_shader_;
    GLuint fragment_shader_;

    GLuint shader_program_;
};

}


#endif // ITOMP_RENDERER_SHADER_PROGRAM_H