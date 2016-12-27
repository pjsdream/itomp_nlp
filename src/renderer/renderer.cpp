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

void Renderer::render(Entity* entity, StaticShader* shader)
{
    TexturedModel* textured_model = entity->getModel();
    RawModel* raw_model = textured_model_->getModel();

    gl_->glBindVertexArray(raw_model->getVAO());

    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    Eigen::Affine3f transformation = entity->getTransformation().cast<float>();
    shader->loadTransformationMatrix(transformation.matrix());

    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glBindTexture(GL_TEXTURE_2D, textured_model->getTexture()->getId());

    gl_->glDrawElements(GL_TRIANGLES, raw_model->getNumVertices(), GL_UNSIGNED_INT, 0);

    gl_->glDisableVertexAttribArray(0);
    gl_->glDisableVertexAttribArray(1);
    gl_->glDisableVertexAttribArray(2);

    gl_->glBindVertexArray(0);
}

void Renderer::render(Entity* entity, LightShader* shader)
{
    TexturedModel* textured_model = entity->getModel();
    RawModel* raw_model = textured_model_->getModel();

    gl_->glBindVertexArray(raw_model->getVAO());

    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    Eigen::Affine3f transformation = entity->getTransformation().cast<float>();
    shader->loadModelTransform(transformation.matrix());

    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glBindTexture(GL_TEXTURE_2D, textured_model->getTexture()->getId());

    gl_->glDrawElements(GL_TRIANGLES, raw_model->getNumVertices(), GL_UNSIGNED_INT, 0);

    gl_->glDisableVertexAttribArray(0);
    gl_->glDisableVertexAttribArray(1);
    gl_->glDisableVertexAttribArray(2);

    gl_->glBindVertexArray(0);
}

void Renderer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();

    gl_->glEnable(GL_DEPTH_TEST);
    gl_->glClearColor(0.5f, 1.0f, 1.0f, 1.0f);
    gl_->glClearDepth(1.0f);

    loader_ = new Loader(this);

    light_shader_ = new LightShader(this);

    std::vector<double> vertices = {
        -0.5, 0.5, 0,
        -0.5, -0.5, 0,
        0.5, -0.5, 0,
        0.5, 0.5, 0,
    };

    std::vector<double> normals = {
        -1, 1, 1,
        -1, -1, 1,
        1, -1, 1,
        1, 1, 1,
    };

    std::vector<double> texture_coords = {
        0, 0, 
        0, 1, 
        1, 1, 
        1, 0, 
    };

    std::vector<int> indices = {
        0, 1, 3,
        3, 1, 2,
    };

    model_ = loader_->createRawModel(vertices, normals, texture_coords, indices);
    texture_ = new ModelTexture( loader_->loadTexture("texture/image.png") );
    textured_model_ = new TexturedModel(model_, texture_);
    entity_ = new Entity(textured_model_, Eigen::Affine3d::Identity());
    entity_->getTransformation().translate( Eigen::Vector3d(0, 0, 0) );

    material_ = new Material(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0.5, 0.5, 0.5));

    mesh_ = new itomp_shape::Mesh();
    mesh_->importDaeFile("../meshes/torso_lift_link.dae");

    light_ = new Light(Eigen::Vector3d(0, 0, 20), Eigen::Vector3d(1, 1, 1));
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
    
    light_shader_->start();

    light_shader_->loadCamera(camera_);
    light_shader_->loadLight(light_);
    light_shader_->loadMaterial(material_);

    render(entity_, light_shader_);

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
