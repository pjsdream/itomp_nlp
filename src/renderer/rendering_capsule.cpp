#include <itomp_nlp/renderer/rendering_capsule.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingCapsule::RenderingCapsule(Renderer* renderer)
    : RenderingShape(renderer)
    , vao_(0)
{
    setCapsule(Eigen::Vector3d(0, 0, -1), 0.5, Eigen::Vector3d(0, 0, 1), 0.5);
}

RenderingCapsule::~RenderingCapsule()
{
    gl_->glDeleteVertexArrays(1, &vao_);
    gl_->glDeleteBuffers(2, &vbos_[0]);
}

void RenderingCapsule::setCapsule(const Eigen::Vector3d& p, double rp, const Eigen::Vector3d& q, double rq)
{
    if (rp > rq)
        setCapsule(q, rq, p, rp);

    need_update_buffer_ = true;

    p_ = p.cast<float>();
    q_ = q.cast<float>();
    rp_ = rp;
    rq_ = rq;
}

void RenderingCapsule::updateBuffers()
{
    AlignedVector<Eigen::Vector3f> vertices;
    AlignedVector<Eigen::Vector3f> normals;
    
    AlignedVector<Eigen::Vector3f> plane_vertices;
    AlignedVector<Eigen::Vector3f> plane_normals;

    transformation_.setIdentity();
    transformation_.block(0, 3, 3, 1) = p_;

    const double d = (p_ - q_).norm();
    if (d <= rq_)
    {
        for (int i=0; i<num_interpolations_ * 2; i++)
        {
            const double t = (double)i / (num_interpolations_ * 2 - 1);
            const double theta = (-M_PI / 2) * (1 - t) + (M_PI / 2) * t;
            const double c = std::cos(theta);
            const double s = std::sin(theta);

            plane_vertices.push_back(rp_ * Eigen::Vector3f(c, 0, s));
            plane_normals.push_back(Eigen::Vector3f(c, 0, s));
        }
    }

    else
    {
        const Eigen::Vector3f z = (q_ - p_) / d;
        Eigen::Vector3f x;
        Eigen::Vector3f y;

        if (std::abs(z(1)) >= 1 - 1e-4) x = Eigen::Vector3f(1, 0, 0).cross(z).normalized();
        else x = Eigen::Vector3f(0, 1, 0).cross(z).normalized();
        y = z.cross(x).normalized();

        transformation_.block(0, 0, 3, 1) = x;
        transformation_.block(0, 1, 3, 1) = y;
        transformation_.block(0, 2, 3, 1) = z;

        const double alpha = std::asin((rq_ - rp_) / d);

        for (int i=0; i<num_interpolations_; i++)
        {
            const double t = (double)i / (num_interpolations_ - 1);
            const double theta = (-M_PI / 2) * (1 - t) + (-alpha) * t;
            const double c = std::cos(theta);
            const double s = std::sin(theta);
            
            plane_vertices.push_back(rp_ * Eigen::Vector3f(c, 0, s));
            plane_normals.push_back(Eigen::Vector3f(c, 0, s));
        }

        for (int i=0; i<num_interpolations_; i++)
        {
            const double t = (double)i / (num_interpolations_ - 1);
            const double theta = (-alpha) * (1. - t) + (M_PI / 2) * t;
            const double c = std::cos(theta);
            const double s = std::sin(theta);
            
            plane_vertices.push_back(Eigen::Vector3f(rq_ * c, 0, d + rq_ * s));
            plane_normals.push_back(Eigen::Vector3f(c, 0, s));
        }
    }

    // rotate
    for (int i=0; i<num_interpolations_ * 2; i++)
    {
        const double t0 = (double)i / num_interpolations_;
        const double theta0 = (M_PI * 2.) * t0;
        const double c0 = std::cos(theta0);
        const double s0 = std::sin(theta0);
        
        const double t1 = (double)(i + 1) / num_interpolations_;
        const double theta1 = (M_PI * 2.) * t1;
        const double c1 = std::cos(theta1);
        const double s1 = std::sin(theta1);

        for (int j=0; j<plane_vertices.size() - 1; j++)
        {
            const Eigen::Vector3f& pv0 = plane_vertices[j];
            const Eigen::Vector3f& pn0 = plane_normals[j];
            const Eigen::Vector3f& pv1 = plane_vertices[j + 1];
            const Eigen::Vector3f& pn1 = plane_normals[j + 1];

            const Eigen::Vector3f v0 = Eigen::Vector3f(c0 * pv0(0), s0 * pv0(0), pv0(2));
            const Eigen::Vector3f n0 = Eigen::Vector3f(c0 * pn0(0), s0 * pn0(0), pn0(2));
            const Eigen::Vector3f v1 = Eigen::Vector3f(c0 * pv1(0), s0 * pv1(0), pv1(2));
            const Eigen::Vector3f n1 = Eigen::Vector3f(c0 * pn1(0), s0 * pn1(0), pn1(2));
            const Eigen::Vector3f v2 = Eigen::Vector3f(c1 * pv0(0), s1 * pv0(0), pv0(2));
            const Eigen::Vector3f n2 = Eigen::Vector3f(c1 * pn0(0), s1 * pn0(0), pn0(2));
            const Eigen::Vector3f v3 = Eigen::Vector3f(c1 * pv1(0), s1 * pv1(0), pv1(2));
            const Eigen::Vector3f n3 = Eigen::Vector3f(c1 * pn1(0), s1 * pn1(0), pn1(2));

            if (j==0)
            {
                vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v3);
                normals .push_back(n0); normals .push_back(n1); normals .push_back(n3);
            }

            else if (j == plane_vertices.size() - 2)
            {
                vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
                normals .push_back(n0); normals .push_back(n1); normals .push_back(n2);
            }

            else
            {
                vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v3);
                normals .push_back(n0); normals .push_back(n1); normals .push_back(n3);
                vertices.push_back(v0); vertices.push_back(v3); vertices.push_back(v2);
                normals .push_back(n0); normals .push_back(n3); normals .push_back(n2);
            }
        }
    }

    if (vao_ == 0)
    {
        gl_->glGenVertexArrays(1, &vao_);
        gl_->glBindVertexArray(vao_);

        vbos_.resize(2);
        gl_->glGenBuffers(2, &vbos_[0]);

        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * (num_triangles_ * 3), (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(0);
    
        gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
        gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * (num_triangles_ * 3), (void*)0, GL_DYNAMIC_DRAW);
        gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        gl_->glEnableVertexAttribArray(1);
    }
    
    gl_->glBindVertexArray(vao_);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[0]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * vertices.size(), &vertices[0]);
    
    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbos_[1]);
    gl_->glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * normals.size(), &normals[0]);
    
    need_update_buffer_ = false;
}

void RenderingCapsule::draw(LightShader* shader)
{
    gl_ = renderer_->getGLFunctions();

    if (need_update_buffer_)
        updateBuffers();

    shader->loadModelTransform(transformation_);
    shader->loadMaterial(material_);

    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(GL_TRIANGLES, 0, num_triangles_ * 3);
}

}
