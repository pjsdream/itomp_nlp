#ifndef ITOMP_NLP_KINECT_H
#define ITOMP_NLP_KINECT_H


#include <vector>

#include <Eigen/Dense>


/**
 * Kinect v2 tutorial: http://homes.cs.washington.edu/~edzhang/tutorials/kinect2/kinect4.html
 */

struct IKinectSensor;
struct IMultiSourceFrameReader;
struct ICoordinateMapper;
struct IMultiSourceFrame;

namespace itomp
{

class Kinect
{
public:

    enum JointType
    {
        JointType_SpineBase	= 0,
        JointType_SpineMid	= 1,
        JointType_Neck	= 2,
        JointType_Head	= 3,
        JointType_ShoulderLeft	= 4,
        JointType_ElbowLeft	= 5,
        JointType_WristLeft	= 6,
        JointType_HandLeft	= 7,
        JointType_ShoulderRight	= 8,
        JointType_ElbowRight	= 9,
        JointType_WristRight	= 10,
        JointType_HandRight	= 11,
        JointType_HipLeft	= 12,
        JointType_KneeLeft	= 13,
        JointType_AnkleLeft	= 14,
        JointType_FootLeft	= 15,
        JointType_HipRight	= 16,
        JointType_KneeRight	= 17,
        JointType_AnkleRight	= 18,
        JointType_FootRight	= 19,
        JointType_SpineShoulder	= 20,
        JointType_HandTipLeft	= 21,
        JointType_ThumbLeft	= 22,
        JointType_HandTipRight	= 23,
        JointType_ThumbRight	= 24,
        JointType_Count	= ( JointType_ThumbRight + 1 ) 
    };
    
    enum TrackingState
    {
        TrackingState_NotTracked	= 0,
        TrackingState_Inferred	= 1,
        TrackingState_Tracked	= 2
    };

private:
    
    template <typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

public:

    Kinect();
    
    int bodyCount();

    bool initialize();

    void update();

    bool isBodyTracked(int id);

    const Eigen::Vector3d& getBodyJointPosition(int body, JointType joint_type);

private:
    
    bool is_initialized_;
    bool initializeDevice();
    
    void updateBodyData(IMultiSourceFrame* frame);

    IKinectSensor* sensor_;             // Kinect sensor
    IMultiSourceFrameReader* reader_;   // Kinect data source
    ICoordinateMapper* mapper_;         // Converts between depth, color, and 3d coordinates

    std::vector<char> is_body_tracked_;

    std::vector<AlignedVector<Eigen::Vector3d> > body_joint_positions_;
    std::vector<std::vector<TrackingState> > body_joint_tracking_states_;
};

}


#endif // ITOMP_NLP_KINECT_H