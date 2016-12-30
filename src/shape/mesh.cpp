#include <itomp_nlp/shape/mesh.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/color4.h>

#include <itomp_nlp/utils/conversion.h>

#include <time.h>


namespace itomp_shape
{

Mesh::Mesh()
    : Shape()
{
}

void Mesh::importDaeFile(const std::string& filename)
{
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    
    // mesh
    // TODO: optimize
    for (int i=0; i<scene->mNumMeshes; i++)
    {
        const aiMesh* mesh = scene->mMeshes[i];

        vertices_.resize(mesh->mNumVertices);
        normals_.resize(mesh->mNumVertices);
        if (mesh->HasTextureCoords(0))
            texture_coords_.resize(mesh->mNumVertices);

        for (int j=0; j<mesh->mNumVertices; j++)
        {
            const aiVector3D& vertex = mesh->mVertices[j];
            const aiVector3D& normal = mesh->mNormals[j];
            const aiVector3D& texture_coord = mesh->mTextureCoords[0][j];

            vertices_[j] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
            normals_[j] = Eigen::Vector3d(normal.x, normal.y, normal.z);
            if (mesh->HasTextureCoords(0))
                texture_coords_[j] = Eigen::Vector2d(texture_coord.x, texture_coord.y);
        }

        triangles_.resize(mesh->mNumFaces);

        for (int j=0; j<mesh->mNumFaces; j++)
        {
            const aiFace& face = mesh->mFaces[j];

            triangles_[j] = Eigen::Vector3i( face.mIndices[0], face.mIndices[1], face.mIndices[2] );
        }
    }
}

}
