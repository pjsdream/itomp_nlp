#ifndef ITOMP_RENDERER_RENDERER_H
#define ITOMP_RENDERER_RENDERER_H


#include <cmath>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <itomp_nlp/renderer/camera.h>

#include <itomp_nlp/renderer/resource_manager.h>

#include <itomp_nlp/renderer/entity.h>

#include <itomp_nlp/renderer/static_shader.h>
#include <itomp_nlp/renderer/light_shader.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>

#include <itomp_nlp/shape/mesh.h>


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

    inline QOpenGLFunctions_4_3_Core* getGLFunctions()
    {
        return gl_;
    }

    int registerMeshFile(const std::string& filename);
    int addEntity(int object_id, const Eigen::Affine3d& transform);

protected:

    virtual void initializeGL();
    virtual void resizeGL(int x, int y);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:

    void renderObject(Object* object, LightShader* shader);
    void renderEntity(Entity* entity, LightShader* shader);

    Camera camera_;

    QOpenGLFunctions_4_3_Core* gl_;
    
    ResourceManager* resource_manager_;

    // shaders
    LightShader* light_shader_;
    std::vector<Light*> lights_;

    // objects
    std::vector<Object*> objects_;

    // entities to be drawn
    std::vector<Entity*> entities_;

    int last_mouse_x_;
    int last_mouse_y_;
};

}


#endif // ITOMP_RENDERER_RENDERER_H
