#include <itomp_nlp/renderer/rendering_mesh.h>

// dae importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


namespace itomp
{

RenderingMesh::RenderingMesh(Renderer* renderer, const std::string& filename)
    : RenderingShape(renderer)
{
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);

    /*
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
    */
}

}
