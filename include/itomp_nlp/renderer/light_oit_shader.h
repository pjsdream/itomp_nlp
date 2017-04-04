#ifndef ITOMP_RENDERER_LIGHT_SHADOW_OIT_SHADER_H
#define ITOMP_RENDERER_LIGHT_SHADOW_OIT_SHADER_H


#include <itomp_nlp/renderer/light_shadow_shader.h>


namespace itomp
{

class LightOITShader : public LightShader
{
private:

    static const std::string vertex_filename_;
    static const std::string fragment_filename_;

    static const int MAX_FRAMEBUFFER_WIDTH = 2048;
    static const int MAX_FRAMEBUFFER_HEIGHT = 2048;

public:

    LightOITShader(Renderer* renderer);

    virtual void start();

protected:
    
    virtual void getAllUniformLocations();

private:

    void initializeOITBuffers();

    // head pointer image and PBO for clearing it
    GLuint  head_pointer_texture_;
    GLuint  head_pointer_clear_buffer_;
    // atomic counter buffer
    GLuint  atomic_counter_buffer_;
    // linked list buffer
    GLuint  linked_list_buffer_;
    GLuint  linked_list_texture_;
};

}


#endif // ITOMP_RENDERER_LIGHT_SHADOW_OIT_SHADER_H