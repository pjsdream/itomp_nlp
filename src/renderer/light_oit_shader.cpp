#include <itomp_nlp/renderer/light_oit_shader.h>

#include <itomp_nlp/renderer/renderer.h>

#include <iostream>


namespace itomp
{

const std::string LightOITShader::vertex_filename_ = "shader/light_oit.vert";
const std::string LightOITShader::fragment_filename_ = "shader/light_oit.frag";


LightOITShader::LightOITShader(Renderer* renderer)
    : LightShader(renderer, vertex_filename_, fragment_filename_)
{
    getAllUniformLocations();

    initializeOITBuffers();
}

void LightOITShader::getAllUniformLocations()
{
}

void LightOITShader::initializeOITBuffers()
{
    // Create head pointer texture
    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glGenTextures(1, &head_pointer_texture_);
    gl_->glBindTexture(GL_TEXTURE_2D, head_pointer_texture_);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, MAX_FRAMEBUFFER_WIDTH, MAX_FRAMEBUFFER_HEIGHT, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
    gl_->glBindTexture(GL_TEXTURE_2D, 0);

    gl_->glBindImageTexture(0, head_pointer_texture_, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);

    // Create buffer for clearing the head pointer texture
    gl_->glGenBuffers(1, &head_pointer_clear_buffer_);
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, head_pointer_clear_buffer_);
    gl_->glBufferData(GL_PIXEL_UNPACK_BUFFER, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * sizeof(GLuint), NULL, GL_STATIC_DRAW);
    GLuint* data = (GLuint*)gl_->glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
    memset(data, 0x00, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * sizeof(GLuint));
    gl_->glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    
    // Create the atomic counter buffer
    gl_->glGenBuffers(1, &atomic_counter_buffer_);
    gl_->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_counter_buffer_);
    gl_->glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint), NULL, GL_DYNAMIC_COPY);
    
    // Create the linked list storage buffer
    gl_->glGenBuffers(1, &linked_list_buffer_);
    gl_->glBindBuffer(GL_TEXTURE_BUFFER, linked_list_buffer_);
    gl_->glBufferData(GL_TEXTURE_BUFFER, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * 3 * (sizeof(float) * 4), NULL, GL_DYNAMIC_COPY);
    gl_->glBindBuffer(GL_TEXTURE_BUFFER, 0);
    
    // Bind it to a texture (for use as a TBO)
    gl_->glGenTextures(1, &linked_list_texture_);
    gl_->glBindTexture(GL_TEXTURE_BUFFER, linked_list_texture_);
    gl_->glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32UI, linked_list_buffer_);
    gl_->glBindTexture(GL_TEXTURE_BUFFER, 0);
    
    gl_->glBindImageTexture(1, linked_list_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
}

void LightOITShader::start()
{
    ShaderProgram::start();
    
    gl_->glActiveTexture(GL_TEXTURE0);

    // Reset atomic counter
    gl_->glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, atomic_counter_buffer_);
    GLuint* data = (GLuint *)gl_->glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_WRITE_ONLY);
    data[0] = 0;
    gl_->glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    // Clear head-pointer image
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, head_pointer_clear_buffer_);
    gl_->glBindTexture(GL_TEXTURE_2D, head_pointer_texture_);
    gl_->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, renderer_->width(), renderer_->height(), GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
    gl_->glBindTexture(GL_TEXTURE_2D, 0);
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

    // Bind head-pointer image for read-write
    gl_->glBindImageTexture(0, head_pointer_texture_, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    // Bind linked-list buffer for write
    gl_->glBindImageTexture(1, linked_list_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);
}

}
