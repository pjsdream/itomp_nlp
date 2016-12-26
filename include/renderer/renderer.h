#ifndef ITOMP_RENDERER_RENDERER_H
#define ITOMP_RENDERER_RENDERER_H


#include <cmath>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <renderer/camera.h>
#include <renderer/renderer_object.h>

namespace itomp_renderer
{

class Renderer : public QOpenGLWidget
{
    Q_OBJECT

private:

    static const int MAX_FRAMEBUFFER_WIDTH = 2048;
    static const int MAX_FRAMEBUFFER_HEIGHT = 2048;

public:

    explicit Renderer(QWidget* parent = 0);
    ~Renderer();

    int addTriangularMeshBuffer(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals, const Eigen::Vector4d& color);
    int addBoxBuffer(const Eigen::Vector3d& half_extents, const Eigen::Vector4d& color);
    
    int addObject(int buffer_id);
    int addObject(int buffer_id, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);
    void setObjectPose(int object_id, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

protected:

    virtual void initializeGL();
    virtual void resizeGL(int x, int y);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:

    void initializeOITBuffers();
    void displayOIT();

    GLuint loadShader(GLuint shader_type, const std::string& filename);
    GLuint linkShaderProgram(const std::vector<GLuint>& shaders);

    Camera camera_;

    QOpenGLFunctions_4_3_Core* gl_;

    GLuint mesh_vertex_shader_;
    GLuint mesh_fragment_shader_;
    GLuint mesh_shader_program_;
    GLuint mesh_shader_location_view_matrix_;
    GLuint mesh_shader_location_projection_matrix_;

    GLuint line_vertex_shader_;
    GLuint line_fragment_shader_;
    GLuint line_shader_program_;
    GLuint line_shader_location_view_matrix_;
    GLuint line_shader_location_projection_matrix_;

    GLuint oit_build_vertex_shader_;
    GLuint oit_build_fragment_shader_;
    GLuint oit_build_shader_program_;
    GLuint oit_build_shader_location_model_matrix_;
    GLuint oit_build_shader_location_view_matrix_;
    GLuint oit_build_shader_location_projection_matrix_;
    GLuint oit_build_shader_location_eye_position_;

    GLuint oit_resolve_vertex_shader_;
    GLuint oit_resolve_fragment_shader_;
    GLuint oit_resolve_shader_program_;

    GLuint oit_head_pointer_texture_;
    GLuint oit_head_pointer_clear_buffer_;
    GLuint oit_atomic_counter_buffer_;
    GLuint oit_linked_list_buffer_;
    GLuint oit_linked_list_texture_;
    GLuint oit_quad_vao_;
    GLuint oit_quad_vbo_;

    // visualizing objects
    std::vector<int> object_ids_;
    std::vector<Eigen::Affine3d> object_transformations_;

    // objects
    std::vector<RendererObjectBuffer*> object_buffers_;

    int last_mouse_x_;
    int last_mouse_y_;
};

}


#endif // ITOMP_RENDERER_RENDERER_H
