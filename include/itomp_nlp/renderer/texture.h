#ifndef ITOMP_RENDERER_TEXTURE_H
#define ITOMP_RENDERER_TEXTURE_H


#include <itomp_nlp/renderer/gl_base.h>


namespace itomp_renderer
{

class Texture
{
public:

    Texture(GLuint id);

    inline GLuint getId()
    {
        return texture_id_;
    }

private:

    GLuint texture_id_;
};

}


#endif // ITOMP_RENDERER_TEXTURE_H