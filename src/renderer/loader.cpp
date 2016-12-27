#include <itomp_nlp/renderer/loader.h>

#include <lodepng.h>

// dae importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


namespace itomp_renderer
{

Loader::Loader(Renderer* renderer)
    : GLBase(renderer)
{
}

TexturedModel* Loader::loadDaeFile(const std::string& filename)
{
    Assimp::Importer importer;
    importer.SetExtraVerbose(false);
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    
    // Material usage
    /*
    aiMaterial* material = scene->mMaterials[0];

    printf("%d\n", material->GetTextureCount(aiTextureType_DIFFUSE));
    printf("%d\n", material->GetTextureCount(aiTextureType_SPECULAR));
    printf("%d\n", material->GetTextureCount(aiTextureType_AMBIENT));

    aiString aipath;
    material->GetTexture(aiTextureType_DIFFUSE, 0, &aipath);
    std::string path = aipath.data;

    aiColor4D ambient;
    material->Get(AI_MATKEY_COLOR_SPECULAR, ambient);
    printf("%s\n", path.c_str());
    printf("%lf %lf %lf %lf\n", ambient.r, ambient.g, ambient.b, ambient.a);
    */

    return 0;
}

RawModel* Loader::createRawModel(const std::vector<double>& positions, const std::vector<int>& indices)
{
    GLuint vao = createVAO();
    bindIndicesBuffer(indices);
    storeDataInAttributeList(0, 3, positions);
    unbindVAO();

    return new RawModel(vao, indices.size());
}

RawModel* Loader::createRawModel(const std::vector<double>& positions, const std::vector<double>& normals, const std::vector<double>& texture_coords, const std::vector<int>& indices)
{
    GLuint vao = createVAO();
    bindIndicesBuffer(indices);
    storeDataInAttributeList(0, 3, positions);
    storeDataInAttributeList(1, 3, normals);
    storeDataInAttributeList(2, 2, texture_coords);
    unbindVAO();

    return new RawModel(vao, indices.size());
}

void Loader::cleanUp()
{
    gl_->glDeleteVertexArrays(vaos_.size(), vaos_.data());
    vaos_.clear();

    gl_->glDeleteBuffers(vbos_.size(), vbos_.data());
    vbos_.clear();

    gl_->glDeleteTextures(textures_.size(), textures_.data());
    textures_.clear();
}

GLuint Loader::createVAO()
{
    GLuint vao;
    gl_->glGenVertexArrays(1, &vao);
    gl_->glBindVertexArray(vao);

    vaos_.push_back(vao);

    return vao;
}

void Loader::storeDataInAttributeList(int attribute_number, int size, const std::vector<double>& data)
{
    GLuint vbo;
    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);

    std::vector<float> buffer = doubleVectorToFloat(data);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * buffer.size(), buffer.data(), GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(attribute_number, size, GL_FLOAT, GL_FALSE, 0, 0);

    vbos_.push_back(vbo);
}

std::vector<float> Loader::doubleVectorToFloat(const std::vector<double>& v)
{
    std::vector<float> r(v.size());
    for (int i=0; i<v.size(); i++)
        r[i] = v[i];
    return r;
}

void Loader::unbindVAO()
{
    gl_->glBindVertexArray(0);
}

void Loader::bindIndicesBuffer(const std::vector<int> indices)
{
    GLuint vbo;
    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);

    gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    vbos_.push_back(vbo);
}

GLuint Loader::loadTexture(const std::string& filename)
{
    // load png file
    std::vector<unsigned char> image;
    unsigned int width, height;
    unsigned int error = lodepng::decode(image, width, height, filename);

    // if there's an error, display it
    if (error)
        fprintf(stderr, "decoder error %u: %s\n", error, lodepng_error_text(error));

    // load image to OpenGL
    GLuint texture;
    gl_->glGenTextures(1, &texture);
    gl_->glBindTexture(GL_TEXTURE_2D, texture);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data());

    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    gl_->glGenerateMipmap(GL_TEXTURE_2D);

    textures_.push_back(texture);
    return texture;
}

}
