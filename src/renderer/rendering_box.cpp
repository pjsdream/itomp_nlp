#include <itomp_nlp/renderer/rendering_box.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingBox::RenderingBox(Renderer* renderer)
    : RenderingShape(renderer)
    , vao_(0)
{
    setSize(Eigen::Vector3d(1, 1, 1));
}

RenderingBox::~RenderingBox()
{
    gl_->glDeleteVertexArrays(1, &vao_);
    gl_->glDeleteBuffers(2, &vbos_[0]);
}

void RenderingBox::setSize(const Eigen::Vector3d& size)
{
    need_update_buffer_ = true;
    size_ = size;
}

void RenderingBox::addFace(AlignedVector<Eigen::Vector3f>& vertices, AlignedVector<Eigen::Vector3f>& normals, const Eigen::Vector3f& normal)
{
    const int zidx = normal(0) != 0 ? 0 : normal(1) != 0 ? 1 : 2;
    const int xidx = (zidx+1)%3;
    const int yidx = (xidx+1)%3;

    static AlignedVector<Eigen::Vector2f> rectangle = 
    {
        Eigen::Vector2f(-1, -1),
        Eigen::Vector2f( 1, -1),
        Eigen::Vector2f( 1,  1),
        Eigen::Vector2f(-1,  1),
    };

    AlignedVector<Eigen::Vector3f> rectangle3d;

    for (int i=0; i<4; i++)
    {
        Eigen::Vector3f v;
        v(xidx) = rectangle[i](0);
        v(yidx) = rectangle[i](1);
        v(zidx) = normal(zidx);

        v(0) *= size_(0) / 2;
        v(1) *= size_(1) / 2;
        v(2) *= size_(2) / 2;

        rectangle3d.push_back(v);
    }

    vertices.push_back(rectangle3d[0]); vertices.push_back(rectangle3d[1]); vertices.push_back(rectangle3d[2]);
    vertices.push_back(rectangle3d[0]); vertices.push_back(rectangle3d[2]); vertices.push_back(rectangle3d[3]);

    for (int i=0; i<6; i++)
        normals.push_back(normal);
}

void RenderingBox::updateBuffers()
{
    AlignedVector<Eigen::Vector3f> vertices;
    AlignedVector<Eigen::Vector3f> normals;
    
    addFace(vertices, normals, Eigen::Vector3f(-1,  0,  0));
    addFace(vertices, normals, Eigen::Vector3f( 1,  0,  0));
    addFace(vertices, normals, Eigen::Vector3f( 0, -1,  0));
    addFace(vertices, normals, Eigen::Vector3f( 0,  1,  0));
    addFace(vertices, normals, Eigen::Vector3f( 0,  0, -1));
    addFace(vertices, normals, Eigen::Vector3f( 0,  0,  1));

    if (vao_ == 0)
    {
        gl_->glGenVertexArrays(1, &vao_);
        gl_->glBindVertexArray(vao_);

        vbos_.resize(2);
        gl_->glGenBuffers(2, &vbos_[0]);

        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 36, (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(0);
    
        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * 36, (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(1);
    }
    
    gl_->glBindVertexArray(vao_);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * 36, &vertices[0]);
    
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * 36, &normals[0]);
    
    need_update_buffer_ = false;
}

void RenderingBox::draw(LightShader* shader)
{
    if (need_update_buffer_)
        updateBuffers();

    shader->loadModelTransform(transform_);
    shader->loadMaterial(material_);

    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_TRIANGLES, 0, 36);
}

}
