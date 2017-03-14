#include <itomp_nlp/renderer/rendering_plane.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingPlane::RenderingPlane(Renderer* renderer)
    : RenderingShape(renderer)
    , vao_(0)
{
}

RenderingPlane::~RenderingPlane()
{
    if (vao_)
    {
        gl_->glDeleteVertexArrays(1, &vao_);
        gl_->glDeleteBuffers(vbos_.size(), &vbos_[0]);
    }
}

void RenderingPlane::updateBuffers()
{
    if (vao_ == 0)
    {
        static float size = 100.f;
        static float vertices[] = 
        {
            -size, -size, 0,
             size, -size, 0,
            -size,  size, 0,
             size,  size, 0,
        };
        static float normals[] = 
        {
            0, 0, 1,
            0, 0, 1,
            0, 0, 1,
            0, 0, 1,
        };
        static float textures[] = 
        {
            -size/2, -size/2,
             size/2, -size/2,
            -size/2,  size/2,
             size/2,  size/2,
        };

        gl_->glGenVertexArrays(1, &vao_);
        gl_->glBindVertexArray(vao_);

        vbos_.resize(3);
        gl_->glGenBuffers(3, &vbos_[0]);

        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 12, (void*)vertices, GL_STATIC_DRAW);
        gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(0);
    
        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 12, (void*)normals, GL_STATIC_DRAW);
        gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(1);
    
        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[2]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8, (void*)textures, GL_STATIC_DRAW);
        gl_->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(2);
    }
}

void RenderingPlane::draw(LightShader* shader)
{
    updateBuffers();

    shader->loadModelTransform(transform_);
    shader->loadMaterial(material_);

    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

}
