#ifndef ITOMP_RENDERER_TEXTURE_H
#define ITOMP_RENDERER_TEXTURE_H


#include <itomp_nlp/renderer/gl_base.h>


namespace itomp
{

class Texture : public GLBase
{
public:

    Texture(Renderer* renderer, const std::string& filename);
    ~Texture();

    inline GLuint getTexture()
    {
        return texture_;
    }

private:

    GLuint texture_;
};

}


#endif // ITOMP_RENDERER_TEXTURE_H