#ifndef ITOMP_RENDERER_TEXTURE_H
#define ITOMP_RENDERER_TEXTURE_H


#include <itomp_nlp/renderer/gl_base.h>


namespace itomp
{

class Texture : public GLBase
{
public:
    
    Texture(Renderer* renderer);
    ~Texture();

    void loadFile(const std::string& filename);

    void setImage(unsigned int width, unsigned int height, const std::vector<unsigned char>& image);

    GLuint getTexture();

private:

    void updateBuffers();

    GLuint texture_;

    unsigned int width_;
    unsigned int height_;
    std::vector<unsigned char> image_;

    bool need_update_buffers_;
};

}


#endif // ITOMP_RENDERER_TEXTURE_H