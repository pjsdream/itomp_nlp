#include <itomp_nlp/renderer/renderer.h>

#include <itomp_nlp/renderer/rendering_shape.h>

#include <QMouseEvent>

#include <iostream>

#include <QTimer>


namespace itomp
{

Renderer::Renderer(QWidget* parent)
    : QOpenGLWidget(parent)
    , opaque_fbo_(0)
{
    opaque_textures_[0] = 0;
    opaque_textures_[1] = 0;

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);

    camera_.setPerspective();
    camera_.lookAt(Eigen::Vector3d(2, 0, 2), Eigen::Vector3d(0, 0, 0));
    
    // DEBUG: timer
    QTimer* timer = new QTimer();
    timer->setInterval(16);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start();
}

Renderer::~Renderer()
{
}

void Renderer::addShape(RenderingShape* shape)
{
    rendering_shapes_.push_back(shape);
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
            break;
        }
    }
}

void Renderer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();
    
    gl_->glEnable(GL_DEPTH_TEST);
    gl_->glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
    gl_->glClearDepth(1.0f);

    gl_->glDisable(GL_CULL_FACE);


    // shaders
    light_shader_ = new LightShader(this);

    normal_shader_ = new NormalShader(this);
    normal_line_length_ = 0.01;

    wireframe_shader_ = new WireframeShader(this);

    color_shader_ = new ColorShader(this);

    light_shadow_shader_ = new LightShadowShader(this);
    
    shadowmap_shader_ = new ShadowmapShader(this);
    shadowmap_point_shader_ = new ShadowmapPointShader(this);

    light_oit_shader_ = new LightOITShader(this);

    oit_resolve_shader_ = new OITResolveShader(this);
    
    // default light
    Light* light;
    light = new Light(Eigen::Vector3d(9, -1, 10));
    light->setDirectional();
    light->setAmbient(Eigen::Vector3f(0.2, 0.2, 0.2));
    light->setDiffuse(Eigen::Vector3f(0.4, 0.4, 0.4));
    light->setSpecular(Eigen::Vector3f(1, 1, 1));
    lights_.push_back(light);

    // point light
    light = new Light(Eigen::Vector3d(2, 0, 2));
    light->setPoint();
    light->setAmbient(Eigen::Vector3f(0.2, 0.2, 0.2));
    light->setDiffuse(Eigen::Vector3f(0.4, 0.4, 0.4));
    light->setSpecular(Eigen::Vector3f(1, 1, 1));
    light->setAttenuation(Eigen::Vector3f(1, 0.045, 0.0075));
    lights_.push_back(light);
}

void Renderer::resizeGL(int w, int h)
{
    gl_->glViewport(0, 0, w, h);

    camera_.setAspect( (double)w / h );

    // recreate opaque fbo
    if (gl_->glIsFramebuffer(opaque_fbo_))
        gl_->glDeleteFramebuffers(1, &opaque_fbo_);
    if (gl_->glIsTexture(opaque_textures_[0]))
        gl_->glDeleteTextures(2, opaque_textures_);

    gl_->glGenFramebuffers(1, &opaque_fbo_);
    gl_->glGenTextures(2, opaque_textures_);

    gl_->glBindFramebuffer(GL_FRAMEBUFFER, opaque_fbo_);

    gl_->glBindTexture(GL_TEXTURE_2D, opaque_textures_[0]);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_FLOAT, NULL);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, opaque_textures_[0], 0);

    gl_->glBindTexture(GL_TEXTURE_2D, opaque_textures_[1]);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, opaque_textures_[1], 0);

    gl_->glBindFramebuffer(GL_FRAMEBUFFER, 0);

    update();
}

void Renderer::paintGL()
{    
    // light direction from camera
    //lights_[0]->setPosition( - camera_.lookAtDirection() );
    
    GLint screen_fbo;
    gl_->glGetIntegerv(GL_FRAMEBUFFER_BINDING, &screen_fbo);

    // shadowmap shader
    shadowmap_shader_->start();

    int didx = 0;
    for (int i=0; i<lights_.size(); i++)
    {
        if (lights_[i]->isDirectional())
        {
            shadowmap_shader_->bindTexture(didx);
            shadowmap_shader_->loadLight(lights_[i]);
    
            for (int j=0; j<rendering_shapes_.size(); j++)
                if (rendering_shapes_[j]->getAlpha() == 1.f)
                    rendering_shapes_[j]->draw(shadowmap_shader_);

            didx++;
        }
    }

    shadowmap_shader_->stop();

    // shadowmap point shader
    shadowmap_point_shader_->start();

    int pidx = 0;
    for (int i=0; i<lights_.size(); i++)
    {
        if (lights_[i]->isPoint())
        {
            shadowmap_point_shader_->bindTexture(pidx);
            shadowmap_point_shader_->loadLight(lights_[i]);
    
            for (int j=0; j<rendering_shapes_.size(); j++)
                if (rendering_shapes_[j]->getAlpha() == 1.f)
                    rendering_shapes_[j]->draw(shadowmap_point_shader_);

            pidx++;
        }
    }

    shadowmap_point_shader_->stop();

    // restore viewport
    gl_->glViewport(0, 0, width(), height());
    
    // render opaque objects first on an off-screen framebuffer
    gl_->glBindFramebuffer(GL_FRAMEBUFFER, opaque_fbo_);
    gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // light shadow shader
    light_shadow_shader_->start();
    light_shadow_shader_->loadCamera(camera_);
    light_shadow_shader_->loadLights(lights_);
    
    didx = 0;
    pidx = 0;
    for (int i=0; i<lights_.size(); i++)
    {
        if (lights_[i]->isDirectional())
        {
            light_shadow_shader_->bindShadowmapTextureDirectional(didx, shadowmap_shader_->getShadowmapTextureId(didx));
            didx++;
        }

        else if (lights_[i]->isPoint())
        {
            light_shadow_shader_->bindShadowmapTexturePoint(pidx, shadowmap_point_shader_->getShadowmapTextureId(pidx));
            pidx++;
        }
    }
    
    for (int i=0; i<rendering_shapes_.size(); i++)
        if (rendering_shapes_[i]->getAlpha() == 1.f)
            rendering_shapes_[i]->draw(light_shadow_shader_);

    light_shadow_shader_->stop();

    // restore to screen framebuffer
    gl_->glBindFramebuffer(GL_FRAMEBUFFER, screen_fbo);
    gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // light oit shader
    light_oit_shader_->start();
    light_oit_shader_->loadCamera(camera_);
    light_oit_shader_->loadLights(lights_);
    
    gl_->glDisable(GL_DEPTH_TEST);
    gl_->glEnable(GL_BLEND);
    gl_->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (int i=0; i<rendering_shapes_.size(); i++)
        if (rendering_shapes_[i]->getAlpha() < 1.f)
            rendering_shapes_[i]->draw(light_oit_shader_);
    
    gl_->glDisable(GL_BLEND);

    light_oit_shader_->stop();

    // oit resolve shader
    oit_resolve_shader_->start();
    oit_resolve_shader_->bindOpaqueTextures(opaque_textures_[0], opaque_textures_[1]);
    oit_resolve_shader_->resolve();
    oit_resolve_shader_->stop();
    gl_->glEnable(GL_DEPTH_TEST);

    // light shader
    /*
    light_shader_->start();
    light_shader_->loadCamera(camera_);
    light_shader_->loadLights(lights_);

    for (int i=0; i<rendering_shapes_.size(); i++)
        rendering_shapes_[i]->draw(light_shader_);

    light_shader_->stop();
    */

    // color shader
    /*
    color_shader_->start();
    color_shader_->loadCamera(camera_);

    for (int i=0; i<rendering_shapes_.size(); i++)
        rendering_shapes_[i]->draw(color_shader_);

    color_shader_->stop();
    */

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
