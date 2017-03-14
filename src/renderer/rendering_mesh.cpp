#include <itomp_nlp/renderer/rendering_mesh.h>

// dae importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <lodepng.h>

#include <itomp_nlp/renderer/material.h>


namespace itomp
{

RenderingMesh::RenderingMesh(Renderer* renderer, Mesh* mesh)
    : RenderingShape(renderer)
{
    // TODO
}

RenderingMesh::RenderingMesh(Renderer* renderer, const std::string& filename)
    : RenderingShape(renderer)
    , filename_(filename)
{
}

void RenderingMesh::initializeBuffers()
{
    gl_->glGenVertexArrays(1, &vao_);
    gl_->glBindVertexArray(vao_);

    material_ = new Material();

    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filename_, aiProcess_Triangulate);

    // mesh
    if (scene->HasMeshes())
    {
        const aiMesh* mesh = scene->mMeshes[0];

        if (mesh->HasPositions())
        {
            GLuint vbo;
            gl_->glGenBuffers(1, &vbo);
            gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
            gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * mesh->mNumVertices, mesh->mVertices, GL_STATIC_DRAW);
            gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
            gl_->glEnableVertexAttribArray(0);

            vbos_.push_back(vbo);
        }

        if (mesh->HasNormals())
        {
            GLuint vbo;
            gl_->glGenBuffers(1, &vbo);
            gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
            gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * mesh->mNumVertices, mesh->mNormals, GL_STATIC_DRAW);
            gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
            gl_->glEnableVertexAttribArray(1);

            vbos_.push_back(vbo);
        }

        if (mesh->HasTextureCoords(0))
        {
            GLuint vbo;
            gl_->glGenBuffers(1, &vbo);
            gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
            gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * mesh->mNumVertices, 0, GL_STATIC_DRAW);

            float* buffer = (float*)gl_->glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
            for (int j=0; j<mesh->mNumVertices; j++)
            {
                *(buffer++) = mesh->mTextureCoords[0][j][0];
                *(buffer++) = 1 - mesh->mTextureCoords[0][j][1];
            }
            gl_->glUnmapBuffer(GL_ARRAY_BUFFER);

            gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
            gl_->glEnableVertexAttribArray(2);
            
            vbos_.push_back(vbo);
        }

        if (mesh->HasFaces())
        {
            GLuint vbo;
            gl_->glGenBuffers(1, &vbo);
            gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);
            gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * 3 * mesh->mNumFaces, 0, GL_STATIC_DRAW);

            int* buffer = (int*)gl_->glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
            for (int j=0; j<mesh->mNumFaces; j++)
            {
                for (int k=0; k<3; k++)
                    *(buffer++) = mesh->mFaces[j].mIndices[k];
            }
            gl_->glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
            
            vbos_.push_back(vbo);

            num_triangles_ = mesh->mNumFaces;
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

        material_->setAmbientColor( Eigen::Vector4f(ambient_color.r, ambient_color.g, ambient_color.b, ambient_color.a) );
        material_->setSpecularColor( Eigen::Vector4f(specular_color.r, specular_color.g, specular_color.b, specular_color.a) );

        if (imported_material->GetTextureCount(aiTextureType_DIFFUSE) >= 1)
        {
            aiString path;

            imported_material->GetTexture(aiTextureType_DIFFUSE, 0, &path);
            
            // get directory and append texture name
            const int idx = filename_.find_last_of("/\\");
            const std::string texture_filename = idx == std::string::npos ? path.C_Str() : filename_.substr(0, idx) + "/" + path.C_Str();

            Texture* texture = new Texture(renderer_);
            texture->loadFile(texture_filename);

            material_->setDiffuseTexture(texture);
        }

        else
        {
            material_->setDiffuseColor( Eigen::Vector4f(0.5, 0.5, 0.5, 1) );
        }
    }

    else
    {
        material_->setDiffuseColor( Eigen::Vector4f(0.5, 0.5, 0.5, 1) );
    }
}

RenderingMesh::~RenderingMesh()
{
    gl_->glDeleteVertexArrays(1, &vao_);
    gl_->glDeleteBuffers(vbos_.size(), &vbos_[0]);
}

void RenderingMesh::draw(LightShader* shader)
{
    if (vao_ == 0)
        initializeBuffers();

    shader->loadMaterial(material_);
    shader->loadModelTransform(transform_);

    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_TRIANGLES, 0, num_triangles_ * 3);
}

}
