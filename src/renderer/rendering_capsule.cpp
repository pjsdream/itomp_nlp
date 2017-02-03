#include <itomp_nlp/renderer/rendering_capsule.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingCapsule::AlignedVector<Eigen::Vector3f> RenderingCapsule::normals_;
std::vector<int> RenderingCapsule::triangles_;
int RenderingCapsule::num_elements_;

int RenderingCapsule::insertNormal(const Eigen::Vector3f& p)
{
    // TODO: currently O(n)
    for (int i=0; i<normals_.size(); i++)
        if (normals_[i] == p)
            return i;

    normals_.push_back(p);
    return normals_.size() - 1;
}

void RenderingCapsule::generateTriangles(int depth, const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2)
{
    if (depth == max_depth_)
    {
        int i0 = insertNormal(p0);
        int i1 = insertNormal(p1);
        int i2 = insertNormal(p2);

        triangles_.push_back(i0);
        triangles_.push_back(i1);
        triangles_.push_back(i2);
    }

    else
    {
        const Eigen::Vector3f p01 = (p0 + p1).normalized();
        const Eigen::Vector3f p12 = (p1 + p2).normalized();
        const Eigen::Vector3f p20 = (p2 + p0).normalized();

        generateTriangles(depth + 1, p0, p01, p20);
        generateTriangles(depth + 1, p1, p12, p01);
        generateTriangles(depth + 1, p2, p20, p12);
        generateTriangles(depth + 1, p01, p12, p20);
    }
}

void RenderingCapsule::initializeNormals()
{
    generateTriangles(0, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f( 1,  0, 0), Eigen::Vector3f( 0,  1, 0));
    generateTriangles(0, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f( 0,  1, 0), Eigen::Vector3f(-1,  0, 0));
    generateTriangles(0, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(-1,  0, 0), Eigen::Vector3f( 0, -1, 0));
    generateTriangles(0, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f( 0, -1, 0), Eigen::Vector3f( 1,  0, 0));
}

RenderingCapsule::RenderingCapsule(Renderer* renderer)
    : RenderingShape(renderer)
    , vao_(0)
{
    if (normals_.empty())
        initializeNormals();
}

void RenderingCapsule::initializeBuffers()
{
    gl_->glGenVertexArrays(1, &vao_);
    gl_->glBindVertexArray(vao_);

    vbos_.resize(3);
    gl_->glGenBuffers(3, &vbos_[0]);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * (normals_.size() * 2), (void*)0, GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    gl_->glEnableVertexAttribArray(0);
    
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * (normals_.size() * 2), (void*)0, GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    gl_->glEnableVertexAttribArray(1);

    std::vector<int> buffer;
    // top
    for (int i=0; i<triangles_.size(); i++)
        buffer.push_back(triangles_[i]);
    // bottom
    for (int i=0; i<triangles_.size(); i++)
        buffer.push_back(normals_.size() + triangles_[i]);

    gl_->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos_[2]);
    gl_->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * buffer.size(), &buffer[0], GL_STATIC_DRAW);

    num_elements_ = buffer.size();

    setCapsule(Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1), 0.5);
}

void RenderingCapsule::setCapsule(const Eigen::Vector3d& p, const Eigen::Vector3d& q, double d)
{
    p_ = p.cast<float>();
    q_ = q.cast<float>();
    d_ = d;
}

void RenderingCapsule::updateBuffers()
{
    const Eigen::Vector3f pf = p_.cast<float>();
    const Eigen::Vector3f qf = q_.cast<float>();

    Eigen::Vector3f n;
    if ((qf-pf).squaredNorm() <= 1e-12)
        n = Eigen::Vector3f(0, 0, 1);
    else n = (qf-pf).normalized();

    Eigen::Vector3f x;
    if (std::abs(n.y()) >= 1 - 1e-4)
        x = Eigen::Vector3f(0, 0, -1).cross(n).normalized();
    else x = Eigen::Vector3f(0, 1, 0).cross(n).normalized();

    const Eigen::Vector3f y = n.cross(x).normalized();

    Eigen::Matrix3f m;
    m.col(0) = x;
    m.col(1) = y;
    m.col(2) = n;

    AlignedVector<Eigen::Vector3f> vertices;
    AlignedVector<Eigen::Vector3f> normals;

    // top
    for (int i=0; i<normals_.size(); i++)
    {
        vertices.push_back(qf + m * normals_[i]);
        normals.push_back(m * normals_[i]);
    }
    // bottom
    for (int i=0; i<normals_.size(); i++)
    {
        vertices.push_back(pf + m * - normals_[i]);
        normals.push_back(m * - normals_[i]);
    }

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * vertices.size(), &vertices[0]);
    
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * normals.size(), &normals[0]);
}

void RenderingCapsule::draw(LightShader* shader)
{
    gl_ = renderer_->getGLFunctions();

    if (vao_ == 0)
        initializeBuffers();

    updateBuffers();

    shader->loadModelTransform(Eigen::Affine3f::Identity().matrix());

    gl_->glBindVertexArray(vao_);
    gl_->glDrawElements(GL_TRIANGLES, num_elements_, GL_UNSIGNED_INT, 0);
}

}
