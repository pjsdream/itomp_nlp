#include <itomp_nlp/renderer/renderer.h>

#include <itomp_nlp/renderer/rendering_shape.h>

#include <QMouseEvent>

#include <iostream>

#include <QTimer>


namespace itomp
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
    
    // DEBUG: timer
    QTimer* timer = new QTimer();
    timer->setInterval(16);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start();
}

Renderer::~Renderer()
{
}

void Renderer::addShape(RenderingShape* shape, ShaderType shader)
{
    rendering_shapes_.push_back(shape);
    shader_types_.push_back(shader);
}

void Renderer::deleteShape(RenderingShape* shape)
{
    // TODO: currently O(n)
    for (int i=0; i<rendering_shapes_.size(); i++)
    {
        if (rendering_shapes_[i] == shape)
        {
            rendering_shapes_[i] = rendering_shapes_[rendering_shapes_.size() - 1];
            rendering_shapes_.pop_back();

            shader_types_[i] = shader_types_[shader_types_.size() - 1];
            shader_types_.pop_back();
            break;
        }
    }
}

int Renderer::registerMeshFile(const std::string& filename)
{
    Object* object = resource_manager_->importFile(filename);
    objects_.push_back(object);
    return objects_.size() - 1;
}

int Renderer::addEntity(int object_id, const Eigen::Affine3d& transform)
{
    Entity* entity = new Entity(objects_[object_id], transform);
    entities_.push_back(entity);
    return entities_.size() - 1;
}

void Renderer::setEntityTransform(int entity_id, const Eigen::Affine3d& transform)
{
    entities_[entity_id]->setTransformation(transform);
}

void Renderer::renderObject(Object* object, LightShader* shader)
{
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    shader->loadModelTransform(transformation.matrix());
    shader->loadMaterial(object->getMaterial());
    object->draw();
}

void Renderer::renderEntity(Entity* entity, LightShader* shader)
{
    Object* object = entity->getObject();
    Eigen::Affine3f transformation = entity->getTransformation().cast<float>();

    shader->loadModelTransform(transformation.matrix());
    shader->loadMaterial(object->getMaterial());
    object->draw();
}

void Renderer::renderEntityNormals(Entity* entity, NormalShader* shader)
{
    Object* object = entity->getObject();
    Eigen::Affine3f transformation = entity->getTransformation().cast<float>();

    shader->loadModelTransform(transformation.matrix());
    object->draw(GL_POINTS);
}

void Renderer::renderEntityWireframe(Entity* entity, WireframeShader* shader)
{
    Object* object = entity->getObject();
    Eigen::Affine3f transformation = entity->getTransformation().cast<float>();

    shader->loadModelTransform(transformation.matrix());
    object->draw();
}

void Renderer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();

    gl_->glEnable(GL_DEPTH_TEST);
    gl_->glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
    gl_->glClearDepth(1.0f);

    resource_manager_ = new ResourceManager(this);

    // shaders
    light_shader_ = new LightShader(this);

    normal_shader_ = new NormalShader(this);
    setNormalLineLength(0.01);

    wireframe_shader_ = new WireframeShader(this);

    color_shader_ = new ColorShader(this);

    // default light
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

    // light shader
    light_shader_->start();
    light_shader_->loadCamera(camera_);
    light_shader_->loadLights(lights_);

    for (int i=0; i<rendering_shapes_.size(); i++)
    {
        if (shader_types_[i] == SHADER_TYPE_LIGHT)
            rendering_shapes_[i]->draw(light_shader_);
    }

    light_shader_->stop();

    // color shader
    color_shader_->start();
    color_shader_->loadCamera(camera_);

    for (int i=0; i<rendering_shapes_.size(); i++)
    {
        if (shader_types_[i] == SHADER_TYPE_COLOR)
            rendering_shapes_[i]->draw(color_shader_);
    }

    color_shader_->stop();

    // normal shader
    /*
    normal_shader_->start();
    normal_shader_->loadCamera(camera_);
    normal_shader_->loadLineLength(normal_line_length_);

    for (int i=0; i<entities_.size(); i++)
        renderEntityNormals(entities_[i], normal_shader_);

    normal_shader_->stop();

    // wireframe shader
    wireframe_shader_->start();
    wireframe_shader_->loadCamera(camera_);
    
    for (int i=0; i<rendering_shapes_.size(); i++)
        renderShape(rendering_shapes_[i], wireframe_shader_);

    wireframe_shader_->stop();
    */
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
