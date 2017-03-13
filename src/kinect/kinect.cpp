#include <itomp_nlp/kinect/kinect.h>


namespace itomp
{

Kinect::Kinect()
    : device_(KinectDevice::getInstance())
{
    initialize();
}

int Kinect::bodyCount()
{
    return device_->bodyCount();
}

bool Kinect::initialize()
{
    return device_->initialize();
}

void Kinect::update()
{
    device_->update();
}

bool Kinect::isBodyTracked(int id)
{
    return device_->isBodyTracked(id);
}

const Eigen::Vector3d& Kinect::getBodyJointPosition(int body, KinectDevice::JointType joint_type)
{
    return device_->getBodyJointPosition(body, joint_type);
}

unsigned int Kinect::getMaxNumPointCloud()
{
    return device_->getMaxNumPointCloud();
}

unsigned int Kinect::getNumPointCloud()
{
    return device_->getNumPointCloud();
}

void Kinect::getGLPointCloudDepths(GLubyte* depth)
{
    device_->getGLPointCloudDepths(depth);
}

void Kinect::getGLPointCloudColors(GLubyte* color)
{
    device_->getGLPointCloudColors(color);
}

}
