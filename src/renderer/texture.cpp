#include <itomp_nlp/renderer/texture.h>

#include <lodepng.h>


namespace itomp
{

Texture::Texture(Renderer* renderer)
    : GLBase(renderer)
    , texture_(0)
    , need_update_buffers_(false)
{
}

Texture::~Texture()
{
    if (texture_ != 0)
        gl_->glDeleteTextures(1, &texture_);
}

void Texture::loadFile(const std::string& filename)
{
    // load png file
    unsigned int error = lodepng::decode(image_, width_, height_, filename);

    // if there's an error, display it
    if (error)
        fprintf(stderr, "decoder error %u: %s\n", error, lodepng_error_text(error));

    else
        need_update_buffers_ = true;
}

void Texture::setImage(unsigned int width, unsigned int height, const std::vector<unsigned char>& image)
{
    width_ = width;
    height_ = height;
    image_ = image;

    need_update_buffers_ = true;
}

void Texture::updateBuffers()
{
    if (texture_ == 0)
    {
        gl_->glGenTextures(1, &texture_);

        need_update_buffers_ = true;
    }

    if (need_update_buffers_)
    {
        gl_->glBindTexture(GL_TEXTURE_2D, texture_);
        gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_.data());
        
        gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        //gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        //gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        gl_->glGenerateMipmap(GL_TEXTURE_2D);

        need_update_buffers_ = false;
    }
}

GLuint Texture::getTexture()
{
    updateBuffers();

    return texture_;
}

}