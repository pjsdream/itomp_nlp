#define _USE_MATH_DEFINES
 
#include <itomp_nlp/renderer/camera.h>


namespace itomp
{

Camera::Camera()
    : eye_(1., 1., 1.)
    , up_(0., 0., 1.)
    , center_(0., 0., 0.)
    , sensitivity_translation_(0.001)
    , sensitivity_rotation_(0.001)
    , sensitivity_zoom_(0.001)
    , fovy_(60.)
    , aspect_(4. / 3.)
    , near_(0.1)
    , far_(1000.)
    , projection_type_(ORTHO)
{
}

void Camera::lookAt(const Eigen::Vector3d& eye, const Eigen::Vector3d center)
{
    eye_ = eye;
    center_ = center;
}

void Camera::translatePixel(int dx, int dy)
{
    const Eigen::Vector3d n = (eye_ - center_).normalized();
    const Eigen::Vector3d u = up_.cross(n).normalized();
    const Eigen::Vector3d v = n.cross(u);

    eye_ += u * sensitivity_translation_ * -dx;
    eye_ -= v * sensitivity_translation_ * -dy;

    center_ += u * sensitivity_translation_ * -dx;
    center_ -= v * sensitivity_translation_ * -dy;
}

void Camera::rotatePixel(int dx, int dy)
{
    const Eigen::Vector3d n = (eye_ - center_).normalized();
    const Eigen::Vector3d u = up_.cross(n).normalized();
    const Eigen::Vector3d v = n.cross(u);

    Eigen::AngleAxisd rotation_x( - sensitivity_rotation_ * dx, up_ );
    Eigen::AngleAxisd rotation_y( - sensitivity_rotation_ * dy, u );

    eye_ = center_ + rotation_x * (eye_ - center_);
    eye_ = center_ + rotation_y * (eye_ - center_);
}

void Camera::zoomPixel(int dx, int dy)
{
    const Eigen::Vector3d n = (eye_ - center_).normalized();
    const Eigen::Vector3d u = up_.cross(n).normalized();
    const Eigen::Vector3d v = n.cross(u);

    const double distance = sensitivity_zoom_ * (-dy);
    const double minimum_distance = 0.01;

    if ((eye_ - center_).norm() - distance >= minimum_distance)
        eye_ -= n * distance;

    else
        eye_ = n * minimum_distance;
}

Eigen::Matrix4d Camera::projectionMatrix() const
{
    switch (projection_type_)
    {
    case ORTHO:
        return ortho();

    case PERSPECTIVE:
        return perspective();
    }
}

Eigen::Matrix4d Camera::viewMatrix() const
{
    const Eigen::Vector3d n = (eye_ - center_).normalized();
    const Eigen::Vector3d u = up_.cross(n).normalized();
    const Eigen::Vector3d v = n.cross(u);

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block(0, 0, 1, 3) = u.transpose();
    mat.block(1, 0, 1, 3) = v.transpose();
    mat.block(2, 0, 1, 3) = n.transpose();
    mat(0, 3) = -eye_.dot(u);
    mat(1, 3) = -eye_.dot(v);
    mat(2, 3) = -eye_.dot(n);

    return mat;
}

Eigen::Matrix4d Camera::perspective() const
{
    const double t = tan((fovy_ / 180. * M_PI) / 2.);

    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat(0, 0) = 1. / (aspect_ * t);
    mat(1, 1) = 1. / t;
    mat(2, 2) = - (near_ + far_) / (far_ - near_);
    mat(2, 3) = - 2. * near_ * far_ / (far_ - near_);
    mat(3, 2) = -1.;

    return mat;
}

Eigen::Matrix4d Camera::ortho() const
{
    double d = (eye_ - center_).norm();

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat(0, 0) = 1. / (aspect_ * d);
    mat(1, 1) = 1. / d;
    mat(2, 2) = -2. / (far_ - near_);
    mat(2, 3) = -(far_ + near_) / (far_ - near_);

    return mat;
}

}
