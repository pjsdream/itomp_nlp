#ifndef ITOMP_SHAPE_MESH_H
#define ITOMP_SHAPE_MESH_H


#include <itomp_nlp/shape/shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class AABB;

class Mesh : public Shape
{
public:

    Mesh();
    
    inline virtual Shape* clone() const
    {
        return new Mesh(*this);
    }

    void setVertices(const std::vector<Eigen::Vector3d>& vertices);
    void setTriangles(const std::vector<Eigen::Vector3i>& triangles);

    AABB getAABB() const;

private:

    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Eigen::Vector3i> triangles_;
};

}


#endif // ITOMP_SHAPE_MESH_H