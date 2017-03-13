#include <itomp_nlp/kinect/kf_human_motion_prediction.h>


namespace itomp
{

KFHumanMotionPrediction::KFHumanMotionPrediction(int body_id)
    : body_id_(body_id)
{
    kinect_ = new Kinect();
}

KFHumanMotionPrediction::~KFHumanMotionPrediction()
{
    delete kinect_;
}

void KFHumanMotionPrediction::predict()
{
    // TODO
    kinect_->update();
}

bool KFHumanMotionPrediction::isBodyPredicted()
{
    // TODO
    return kinect_->isBodyTracked(body_id_);
}

Eigen::Vector3d KFHumanMotionPrediction::getPredictedBodyJointPosition(double time, KinectDevice::JointType joint_type)
{
    // TODO
    return kinect_->getBodyJointPosition(body_id_, joint_type);
}

}
