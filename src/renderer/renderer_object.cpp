#include <renderer/renderer_object.h>


namespace itomp_renderer
{

RendererObjectBuffer::RendererObjectBuffer(QOpenGLFunctions_4_3_Core* gl)
    : gl_(gl)
{
}

void RendererObjectBuffer::setTriangularMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals, const Eigen::Vector4d& color)
{
    num_vertices_ = vertices.size();
    draw_type_ = GL_TRIANGLES;

    // buffer setup
    const int buffer_size = vertices.size() * sizeof(float) * 10;
    float* buffer = new float[ buffer_size ];

    int index = 0;
    for (int i=0; i<vertices.size(); i++)
    {
        buffer[index++] = vertices[i](0);
        buffer[index++] = vertices[i](1);
        buffer[index++] = vertices[i](2);
        buffer[index++] = normals[i](0);
        buffer[index++] = normals[i](1);
        buffer[index++] = normals[i](2);
        buffer[index++] = color(0);
        buffer[index++] = color(1);
        buffer[index++] = color(2);
        buffer[index++] = color(3);
    }

    gl_->glGenVertexArrays(1, &vao_);
    gl_->glBindVertexArray(vao_);

    gl_->glGenBuffers(1, &vbo_);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_->glBufferData(GL_ARRAY_BUFFER, buffer_size, buffer, GL_STATIC_DRAW);

    gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 0));
    gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 3));
    gl_->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 6));
    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    delete buffer;
}

void RendererObjectBuffer::setBox(const Eigen::Vector3d& half_extents, const Eigen::Vector4d& color)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3d> normals;

    const double& x = half_extents(0);
    const double& y = half_extents(1);
    const double& z = half_extents(2);

    vertices.push_back(Eigen::Vector3d(-x, -y,  z));
    vertices.push_back(Eigen::Vector3d( x, -y,  z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d(-x, -y,  z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d(-x,  y,  z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(0, 0, 1));

    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y, -z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(0, 0, -1));

    vertices.push_back(Eigen::Vector3d(-x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d(-x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d(-x,  y,  z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(0, 1, 0));

    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d( x, -y,  z));
    vertices.push_back(Eigen::Vector3d( x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x, -y,  z));
    vertices.push_back(Eigen::Vector3d( x, -y,  z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(0, -1, 0));

    vertices.push_back(Eigen::Vector3d( x, -y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d( x, -y, -z));
    vertices.push_back(Eigen::Vector3d( x,  y,  z));
    vertices.push_back(Eigen::Vector3d( x, -y,  z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(1, 0, 0));

    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x,  y,  z));
    vertices.push_back(Eigen::Vector3d(-x,  y, -z));
    vertices.push_back(Eigen::Vector3d(-x, -y, -z));
    vertices.push_back(Eigen::Vector3d(-x, -y,  z));
    vertices.push_back(Eigen::Vector3d(-x,  y,  z));
    for (int i=0; i<6; i++)
        normals.push_back(Eigen::Vector3d(-1, 0, 0));

    setTriangularMesh(vertices, normals, color);
}

void RendererObjectBuffer::draw()
{
    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(draw_type_, 0, num_vertices_);
}

}
