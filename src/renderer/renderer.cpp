#include <renderer/renderer.h>

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

void Renderer::renderTexturedModel(TexturedModel* model)
{
    RawModel* raw_model = textured_model_->getModel();

    gl_->glBindVertexArray(raw_model->getVAO());

    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);

    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glBindTexture(GL_TEXTURE_2D, model->getTexture()->getId());

    gl_->glDrawElements(GL_TRIANGLES, raw_model->getNumVertices(), GL_UNSIGNED_INT, 0);

    gl_->glDisableVertexAttribArray(0);
    gl_->glDisableVertexAttribArray(1);

    gl_->glBindVertexArray(0);
}

void Renderer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();

    gl_->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    loader_ = new Loader(this);

    static_shader_ = new StaticShader(this);

    std::vector<double> vertices = {
        -0.5, 0.5, 0,
        -0.5, -0.5, 0,
        0.5, -0.5, 0,
        0.5, 0.5, 0,
    };

    std::vector<int> indices = {
        0, 1, 3,
        3, 1, 2,
    };

    std::vector<double> texture_coords = {
        0, 0, 
        0, 1, 
        1, 1, 
        1, 0, 
    };

    model_ = loader_->createRawModel(vertices, texture_coords, indices);
    texture_ = new ModelTexture( loader_->loadTexture("texture/image.png") );
    textured_model_ = new TexturedModel(model_, texture_);
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

    static_shader_->start();
    renderTexturedModel(textured_model_);
    static_shader_->stop();
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
