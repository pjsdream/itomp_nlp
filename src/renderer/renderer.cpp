#include <itomp_nlp/renderer/renderer.h>

#include <QMouseEvent>

#include <iostream>


namespace itomp_renderer
{

Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);

    camera_.setOrtho();
}

Renderer::~Renderer()
{
}

void Renderer::renderObject(Object* object, LightShader* shader)
{
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    shader->loadModelTransform(transformation.matrix());

    light_shader_->loadMaterial(object->getMaterial());
    object->draw();
}

void Renderer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();

    gl_->glEnable(GL_DEPTH_TEST);
    gl_->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    gl_->glClearDepth(1.0f);

    resource_manager_ = new ResourceManager(this);

    light_shader_ = new LightShader(this);

    object_ = new Object(this);
    object_ = resource_manager_->importDaeFile("../meshes/base_link.dae");

    Light* light;
    
    light = new Light(Eigen::Vector3d(-1, 0, 0));
    light->setDiffuseColor(Eigen::Vector4f(1, 1, 1, 1));
    light->setSpecularColor(Eigen::Vector4f(0, 0, 0, 1));
    lights_.push_back(light);
}

void Renderer::resizeGL(int w, int h)
{
    gl_->glViewport(0, 0, w, h);

    camera_.setAspect( (double)w / h );
    update();
}

void Renderer::paintGL()
{
    gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // light direction from camera
    lights_[0]->setPosition( - camera_.lookAtDirection() );

    light_shader_->start();
    light_shader_->loadCamera(camera_);
    light_shader_->loadLights(lights_);

    renderObject(object_, light_shader_);

    light_shader_->stop();
}

void Renderer::mousePressEvent(QMouseEvent* event)
{
    last_mouse_x_ = event->x();
    last_mouse_y_ = event->y();
}

void Renderer::mouseMoveEvent(QMouseEvent* event)
{
    const int x = event->x();
    const int y = event->y();
    const int dx = x - last_mouse_x_;
    const int dy = y - last_mouse_y_;

    last_mouse_x_ = x;
    last_mouse_y_ = y;

    switch (event->buttons())
    {
    case Qt::LeftButton:
        camera_.rotatePixel(dx, dy);
        update();
        break;

    case Qt::RightButton:
        camera_.translatePixel(dx, dy);
        update();
        break;

    case (int)Qt::LeftButton | (int)Qt::RightButton:
        camera_.zoomPixel(dx, dy);
        update();
        break;
    }
}

}
