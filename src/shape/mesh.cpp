#include <itomp_nlp/shape/mesh.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>


namespace itomp_shape
{

Mesh::Mesh()
{
}

void Mesh::importDaeFile(const std::string& filename)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);

    for (int i=0; i<scene->mNumMeshes; i++)
    {
        const aiMesh* mesh = scene->mMeshes[i];

        for (int j=0; j<mesh->mNumVertices; j++)
        {
            const aiVector3D& vertex = mesh->mVertices[j];
            const aiVector3D& normal = mesh->mNormals[j];
            const aiVector3D& texture_coord = mesh->mTextureCoords[0][j];

            /*
            printf("%lf %lf %lf  %lf %lf %lf  %lf %lf %lf\n",
                   vertex.x, vertex.y, vertex.z,
                   normal.x, normal.y, normal.z,
                   texture_coord.x, texture_coord.y, texture_coord.z);
                   */

            //vertices_.push_back
        }
    }
}

}
