#ifndef ITOMP_NLP_KINECT_H
#define ITOMP_NLP_KINECT_H


#include <itomp_nlp/kinect/kinect_device.h>


namespace itomp
{

class Kinect
{
public:

    Kinect();
    
    int bodyCount();

    void update();

    bool isBodyTracked(int id);

    const Eigen::Vector3d& getBodyJointPosition(int body, KinectDevice::JointType joint_type);
    
    unsigned int getMaxNumPointCloud();
    unsigned int getNumPointCloud();
    void getGLPointCloudDepths(GLubyte* depth);
    void getGLPointCloudColors(GLubyte* color);

private:
    
    bool initialize();

    KinectDevice* device_;
};

}


#endif // ITOMP_NLP_KINECT_H