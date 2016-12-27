#ifndef ITOMP_RENDERER_RAW_MODEL_LOADER_H
#define ITOMP_RENDERER_RAW_MODEL_LOADER_H


#include <vector>

#include <renderer/raw_model.h>
#include <renderer/textured_model.h>
#include <renderer/gl_base.h>


namespace itomp_renderer
{

class Loader : public GLBase
{
public:

    Loader(Renderer* renderer);

    RawModel* createRawModel(const std::vector<double>& positions, const std::vector<int>& indices);
    RawModel* createRawModel(const std::vector<double>& positions, const std::vector<double> texture_coords, const std::vector<int>& indices);

    GLuint loadTexture(const std::string& filename);

    void cleanUp();

private:

    GLuint createVAO();
    void storeDataInAttributeList(int attribute_number, int size, const std::vector<double>& data);
    std::vector<float> doubleVectorToFloat(const std::vector<double>& v);
    void unbindVAO();
    void bindIndicesBuffer(const std::vector<int> indices);

    std::vector<GLuint> vaos_;
    std::vector<GLuint> vbos_;
    std::vector<GLuint> textures_;
};

}


#endif // ITOMP_RENDERER_RAW_MODEL_LOADER_H