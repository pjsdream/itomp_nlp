#include <itomp_nlp/renderer/texture.h>

#include <lodepng.h>


namespace itomp
{

Texture::Texture(Renderer* renderer, const std::string& filename)
    : GLBase(renderer)
{
    // load png file
    std::vector<unsigned char> image;
    unsigned int width, height;
    unsigned int error = lodepng::decode(image, width, height, filename);

    // if there's an error, display it
    if (error)
        fprintf(stderr, "decoder error %u: %s\n", error, lodepng_error_text(error));

    // load image to OpenGL
    gl_->glGenTextures(1, &texture_);
    gl_->glBindTexture(GL_TEXTURE_2D, texture_);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data());

    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    gl_->glGenerateMipmap(GL_TEXTURE_2D);
}

Texture::~Texture()
{
    gl_->glDeleteTextures(1, &texture_);
}

}