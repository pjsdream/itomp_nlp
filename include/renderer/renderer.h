#ifndef ITOMP_RENDERER_RENDERER_H
#define ITOMP_RENDERER_RENDERER_H


#include <cmath>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <renderer/camera.h>

#include <renderer/loader.h>

#include <renderer/entity.h>

#include <renderer/static_shader.h>


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

protected:

    virtual void initializeGL();
    virtual void resizeGL(int x, int y);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:

    void render(Entity* model, StaticShader* shader);

    Camera camera_;

    QOpenGLFunctions_4_3_Core* gl_;
    
    // loader
    Loader* loader_;

    // models
    RawModel* model_;
    ModelTexture* texture_;
    TexturedModel* textured_model_;
    Entity* entity_;

    // shaders
    StaticShader* static_shader_;

    int last_mouse_x_;
    int last_mouse_y_;
};

}


#endif // ITOMP_RENDERER_RENDERER_H
