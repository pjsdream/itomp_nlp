#ifndef ITOMP_NLP_KF_HUMAN_MOTION_PREDICTION_H
#define ITOMP_NLP_KF_HUMAN_MOTION_PREDICTION_H


#include <itomp_nlp/kinect/kinect.h>

#include <vector>


namespace itomp
{

class KFHumanMotionPrediction
{
public:

    KFHumanMotionPrediction(int body_id);
    ~KFHumanMotionPrediction();
    
    void predict();

    bool isBodyPredicted();
    Eigen::Vector3d getPredictedBodyJointPosition(double time, KinectDevice::JointType joint_type);

private:

    const int body_id_;

    Kinect* kinect_;
};

}


#endif // ITOMP_NLP_KF_HUMAN_MOTION_PREDICTION_H