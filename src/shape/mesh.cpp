#include <itomp_nlp/shape/mesh.h>

#include <itomp_nlp/shape/aabb.h>


namespace itomp
{

Mesh::Mesh()
    : Shape()
{
}

void Mesh::setVertices(const std::vector<Eigen::Vector3d>& vertices)
{
    vertices_ = vertices;
}

void Mesh::setTriangles(const std::vector<Eigen::Vector3i>& triangles)
{
    triangles_ = triangles;
}

AABB Mesh::getAABB() const
{
    Eigen::Vector3d lower = vertices_[0];
    Eigen::Vector3d upper = vertices_[0];

    for (int i=1; i<vertices_.size(); i++)
    {
        const Eigen::Vector3d& vertex = vertices_[i];

        if (lower(0) > vertex(0)) lower(0) = vertex(0);
        if (lower(1) > vertex(1)) lower(1) = vertex(1);
        if (lower(2) > vertex(2)) lower(2) = vertex(2);

        if (upper(0) < vertex(0)) upper(0) = vertex(0);
        if (upper(1) < vertex(1)) upper(1) = vertex(1);
        if (upper(2) < vertex(2)) upper(2) = vertex(2);
    }

    return AABB(lower, upper);
}

}
