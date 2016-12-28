#ifndef ITOMP_RENDERER_RESOURCE_MANAGER_H
#define ITOMP_RENDERER_RESOURCE_MANAGER_H


#include <vector>

#include <itomp_nlp/renderer/texture.h>
#include <itomp_nlp/renderer/gl_base.h>
#include <itomp_nlp/renderer/object.h>


namespace itomp_renderer
{

class ResourceManager : public GLBase
{
public:

    ResourceManager(Renderer* renderer);

    Object* importFile(const std::string& filename);

    Texture* loadTexture(const std::string& filename);

    void cleanUp();

private:

    GLuint createVAO();
    void unbindVAO();
    
    void storeInFloatBuffer(int attribute_location, int attribute_size, int bytes, const void* buffer);
    void storeInElementBuffer(int bytes, const void* buffer);

    void* mapArrayBuffer(int bytes);
    void unmapArrayBuffer();

    void* mapElementBuffer(int bytes);
    void unmapElementBuffer();

    std::vector<GLuint> vaos_;
    std::vector<GLuint> vbos_;
    std::vector<GLuint> textures_;
};

}


#endif // ITOMP_RENDERER_RESOURCE_MANAGER_H