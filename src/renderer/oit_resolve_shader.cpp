#include <itomp_nlp/renderer/oit_resolve_shader.h>

#include <itomp_nlp/renderer/renderer.h>

#include <iostream>


namespace itomp
{

const std::string OITResolveShader::vertex_filename_ = "shader/resolve_lists.vert";
const std::string OITResolveShader::fragment_filename_ = "shader/resolve_lists.frag";


OITResolveShader::OITResolveShader(Renderer* renderer)
    : ShaderProgram(renderer, vertex_filename_, fragment_filename_)
    , vao_(0)
{
    bindAttributes();
}

OITResolveShader::~OITResolveShader()
{
    gl_->glDeleteVertexArrays(1, &vao_);
    gl_->glDeleteBuffers(1, &vbo_);
}

void OITResolveShader::bindAttributes()
{
    bindAttribute(0, "position");
}

void OITResolveShader::initializeQuadBuffer()
{
    gl_->glGenVertexArrays(1, &vao_);
    gl_->glBindVertexArray(vao_);

    static const GLfloat quad_verts[] =
    {
        -1.0f, -1.0f,
         1.0f, -1.0f,
        -1.0f,  1.0f,
         1.0f,  1.0f,
    };

    gl_->glGenBuffers(1, &vbo_);
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(quad_verts), quad_verts, GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    gl_->glEnableVertexAttribArray(0);
}

void OITResolveShader::resolve()
{
    if (vao_ == 0)
        initializeQuadBuffer();

    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

}
