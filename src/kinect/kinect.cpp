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

}
