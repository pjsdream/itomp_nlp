#ifndef ITOMP_RENDERER_RENDERER_H
#define ITOMP_RENDERER_RENDERER_H


#include <cmath>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <itomp_nlp/renderer/camera.h>

#include <itomp_nlp/renderer/static_shader.h>
#include <itomp_nlp/renderer/light_shader.h>
#include <itomp_nlp/renderer/color_shader.h>
#include <itomp_nlp/renderer/normal_shader.h>
#include <itomp_nlp/renderer/wireframe_shader.h>
#include <itomp_nlp/renderer/shadowmap_shader.h>
#include <itomp_nlp/renderer/light_shadow_shader.h>
#include <itomp_nlp/renderer/light.h>
#include <itomp_nlp/renderer/material.h>

#include <itomp_nlp/shape/mesh.h>


namespace itomp
{

class RenderingShape;

class Renderer : public QOpenGLWidget
{
    Q_OBJECT

public:

    explicit Renderer(QWidget* parent = 0);
    ~Renderer();

    inline QOpenGLFunctions_4_3_Core* getGLFunctions()
    {
        return gl_;
    }

    void addShape(RenderingShape* shape);
    void deleteShape(RenderingShape* shape);

protected:

    virtual void initializeGL();
    virtual void resizeGL(int x, int y);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:

    Camera camera_;

    QOpenGLFunctions_4_3_Core* gl_;
    
    // shaders
    LightShader* light_shader_;
    std::vector<Light*> lights_;
    NormalShader* normal_shader_;
    double normal_line_length_;
    WireframeShader* wireframe_shader_;
    ColorShader* color_shader_;
    
    LightShadowShader* light_shadow_shader_;
    ShadowmapShader* shadowmap_shader_;

    // rendering shapes
    std::vector<RenderingShape*> rendering_shapes_;

    int last_mouse_x_;
    int last_mouse_y_;
};

}


#endif // ITOMP_RENDERER_RENDERER_H
