#include <itomp_nlp/renderer/rendering_kinect_points.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingKinectPoints::RenderingKinectPoints(Renderer* renderer)
    : RenderingShape(renderer)
    , vao_(0)
{
    kinect_ = new Kinect();
}

RenderingKinectPoints::~RenderingKinectPoints()
{
    gl_->glDeleteVertexArrays(1, &vao_);
    gl_->glDeleteBuffers(2, vbos_);
}

void RenderingKinectPoints::updateBuffers()
{
    if (vao_ == 0)
    {
        gl_->glGenVertexArrays(1, &vao_);
        gl_->glBindVertexArray(vao_);

        gl_->glGenBuffers(2, vbos_);

        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * kinect_->getMaxNumPointCloud(), (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(0);

        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * kinect_->getMaxNumPointCloud(), (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(1);
    }

    gl_->glBindVertexArray(vao_);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
    GLubyte* buffer = (GLubyte*)gl_->glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    kinect_->getGLPointCloudDepths(buffer);
    gl_->glUnmapBuffer(GL_ARRAY_BUFFER);
    
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
    buffer = (GLubyte*)gl_->glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    kinect_->getGLPointCloudColors(buffer);
    gl_->glUnmapBuffer(GL_ARRAY_BUFFER);
    
}

void RenderingKinectPoints::draw(ColorShader* shader)
{
    kinect_->update();

    updateBuffers();
    
    shader->loadModelTransform(transform_);

    gl_->glPointSize(5.0);
    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_POINTS, 0, kinect_->getNumPointCloud());
}

}
