#include <itomp_nlp/shape/mesh_loader.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/color4.h>

#include <itomp_nlp/utils/conversion.h>

#include <time.h>


namespace itomp
{

Mesh* MeshLoader::loadMeshFile(const std::string& filename)
{
    // TODO: prevent loading the same file multiple times
    // current code does not support such functionality
    
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;

    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    
    // mesh
    // TODO: optimize
    for (int i=0; i<scene->mNumMeshes; i++)
    {
        const aiMesh* mesh = scene->mMeshes[i];

        vertices.resize(mesh->mNumVertices);

        for (int j=0; j<mesh->mNumVertices; j++)
        {
            const aiVector3D& vertex = mesh->mVertices[j];
            vertices[j] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
        }

        triangles.resize(mesh->mNumFaces);

        for (int j=0; j<mesh->mNumFaces; j++)
        {
            const aiFace& face = mesh->mFaces[j];
            triangles[j] = Eigen::Vector3i( face.mIndices[0], face.mIndices[1], face.mIndices[2] );
        }
    }

    Mesh* mesh = new Mesh();
    mesh->setVertices(vertices);
    mesh->setTriangles(triangles);

    return mesh;
}

}
