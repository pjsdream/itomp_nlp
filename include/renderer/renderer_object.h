#ifndef ITOMP_RENDERER_RENDERER_OBJECT_BUFFER_H
#define ITOMP_RENDERER_RENDERER_OBJECT_BUFFER_H


#include <QOpenGLFunctions_4_3_Core>

#include <Eigen/Dense>


namespace itomp_renderer
{

class RendererObjectBuffer
{
public:

    RendererObjectBuffer(QOpenGLFunctions_4_3_Core* gl);

    void setTriangularMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals, const Eigen::Vector4d& color);
    void setBox(const Eigen::Vector3d& half_extents, const Eigen::Vector4d& color);

    void draw();

private:

    QOpenGLFunctions_4_3_Core* gl_;

    GLuint vao_;
    GLuint vbo_;
    int num_vertices_;
    GLuint draw_type_;
};

}


#endif // ITOMP_RENDERER_RENDERER_OBJECT_BUFFER_H