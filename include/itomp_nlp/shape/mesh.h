#ifndef ITOMP_SHAPE_MESH_H
#define ITOMP_SHAPE_MESH_H


#include <itomp_nlp/shape/shape.h>

#include <string>
#include <vector>

#include <Eigen/Dense>


namespace itomp_shape
{

class Mesh : public Shape
{
public:

    Mesh();

    void importDaeFile(const std::string& filename);

private:

    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Eigen::Vector3d> normals_;
    std::vector<Eigen::Vector2d> texture_coords_;
    std::vector<Eigen::Vector3i> traingles_;
};

}


#endif // ITOMP_SHAPE_MESH_H