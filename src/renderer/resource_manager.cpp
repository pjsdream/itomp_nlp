#include <itomp_nlp/renderer/resource_manager.h>

#include <lodepng.h>

// dae importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <itomp_nlp/utils/timing.h>


namespace itomp_renderer
{

ResourceManager::ResourceManager(Renderer* renderer)
    : GLBase(renderer)
{
}

void ResourceManager::cleanUp()
{
    gl_->glDeleteVertexArrays(vaos_.size(), vaos_.data());
    vaos_.clear();

    gl_->glDeleteBuffers(vbos_.size(), vbos_.data());
    vbos_.clear();

    gl_->glDeleteTextures(textures_.size(), textures_.data());
    textures_.clear();
}

GLuint ResourceManager::createVAO()
{
    GLuint vao;
    gl_->glGenVertexArrays(1, &vao);
    gl_->glBindVertexArray(vao);

    vaos_.push_back(vao);

    return vao;
}

void ResourceManager::unbindVAO()
{
    gl_->glBindVertexArray(0);
}

Object* ResourceManager::importFile(const std::string& filename)
{
    Object* object = new Object(getRenderer());
    Material* material = new Material();

    object->setMaterial(material);
    object->setVAO( createVAO() );

    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);

    // mesh
    if (scene->HasMeshes())
    {
        const aiMesh* mesh = scene->mMeshes[0];

        if (mesh->HasPositions())
            storeInFloatBuffer(0, 3, sizeof(float) * 3 * mesh->mNumVertices, mesh->mVertices);

        if (mesh->HasNormals())
            storeInFloatBuffer(1, 3, sizeof(float) * 3 * mesh->mNumVertices, mesh->mNormals);

        if (mesh->HasTextureCoords(0))
        {
            float* buffer = (float*)mapArrayBuffer(sizeof(float) * 2 * mesh->mNumVertices);

            for (int j=0; j<mesh->mNumVertices; j++)
            {
                *(buffer++) = mesh->mTextureCoords[0][j][0];
                *(buffer++) = 1 - mesh->mTextureCoords[0][j][1];
            }

            unmapArrayBuffer();

            gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
        }

        if (mesh->HasFaces())
        {
            int* buffer = (int*)mapElementBuffer(sizeof(int) * 3 * mesh->mNumFaces);

            for (int j=0; j<mesh->mNumFaces; j++)
            {
                for (int k=0; k<3; k++)
                    *(buffer++) = mesh->mFaces[j].mIndices[k];
            }

            unmapElementBuffer();

            object->setNumVertices(3 * mesh->mNumFaces);
        }
    }

    // color
    if (scene->HasMaterials())
    {
        const aiMaterial* imported_material = scene->mMaterials[0];

        aiColor4D ambient_color;
        aiColor4D specular_color;
        float shininess;

        imported_material->Get(AI_MATKEY_COLOR_AMBIENT, ambient_color);
        imported_material->Get(AI_MATKEY_COLOR_SPECULAR, specular_color);
        imported_material->Get(AI_MATKEY_SHININESS, shininess);

        material->setAmbientColor( Eigen::Vector4f(ambient_color.r, ambient_color.g, ambient_color.b, ambient_color.a) );
        material->setSpecularColor( Eigen::Vector4f(specular_color.r, specular_color.g, specular_color.b, specular_color.a) );

        if (imported_material->GetTextureCount(aiTextureType_DIFFUSE) >= 1)
        {
            aiString path;

            imported_material->GetTexture(aiTextureType_DIFFUSE, 0, &path);
            
            const int idx = filename.find_last_of("/\\");
            const std::string texture_filename = idx == std::string::npos ? path.C_Str() : filename.substr(0, idx) + "/" + path.C_Str();

            material->setDiffuseTexture( loadTexture(texture_filename) );
        }

        else
        {
            material->setDiffuseColor( Eigen::Vector4f(0.5, 0.5, 0.5, 1) );
        }
    }

    else
    {
        // TODO: default color
    }

    return object;
}

void ResourceManager::storeInFloatBuffer(int attribute_location, int attribute_size, int bytes, const void* buffer)
{
    GLuint vbo;

    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ARRAY_BUFFER, bytes, buffer, GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(attribute_location, attribute_size, GL_FLOAT, GL_FALSE, 0, 0);

    vbos_.push_back(vbo);
}

void ResourceManager::storeInElementBuffer(int bytes, const void* buffer)
{
    GLuint vbo;

    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, bytes, buffer, GL_STATIC_DRAW);

    vbos_.push_back(vbo);
}

void* ResourceManager::mapArrayBuffer(int bytes)
{
    GLuint vbo;

    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ARRAY_BUFFER, bytes, 0, GL_STATIC_DRAW);

    vbos_.push_back(vbo);

    return gl_->glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
}

void ResourceManager::unmapArrayBuffer()
{
    gl_->glUnmapBuffer(GL_ARRAY_BUFFER);
}

void* ResourceManager::mapElementBuffer(int bytes)
{
    GLuint vbo;

    gl_->glGenBuffers(1, &vbo);
    gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, bytes, 0, GL_STATIC_DRAW);
    
    vbos_.push_back(vbo);

    return gl_->glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
}

void ResourceManager::unmapElementBuffer()
{
    gl_->glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
}

Texture* ResourceManager::loadTexture(const std::string& filename)
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

    return new Texture(texture);
}

}
