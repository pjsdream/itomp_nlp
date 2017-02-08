#ifndef ITOMP_OPTIMIZATION_DYNAMIC_KF_HUMAN_OBSTACLE_H
#define ITOMP_OPTIMIZATION_DYNAMIC_KF_HUMAN_OBSTACLE_H


#include <itomp_nlp/optimization/dynamic_obstacle.h>

#include <itomp_nlp/kinect/kf_human_motion_prediction.h>

#include <itomp_nlp/shape/capsule2.h>


namespace itomp
{

class DynamicKFHumanObstacle : public DynamicObstacle
{
public:

    DynamicKFHumanObstacle(int body_id);
    ~DynamicKFHumanObstacle();

    inline void setCameraTransform(const Eigen::Affine3d& camera_transform)
    {
        camera_transform_ = camera_transform;
    }

    virtual std::vector<Shape*> getShapes(double t);

private:

    static const std::vector<double> radii_;
    static const std::vector<std::pair<KinectDevice::JointType, KinectDevice::JointType> > edges_;

    const int body_id_;

    Eigen::Affine3d camera_transform_;

    KFHumanMotionPrediction* predictor_;

    std::vector<Capsule2*> capsules_;
};

}


#endif // ITOMP_OPTIMIZATION_DYNAMIC_KF_HUMAN_OBSTACLE_H