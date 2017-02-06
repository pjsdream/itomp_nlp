#include <itomp_nlp/kinect/kinect.h>

#include <Kinect.h>


namespace itomp
{

Kinect::Kinect()
{
    is_body_tracked_.resize(BODY_COUNT, false);
    body_joint_positions_.resize(BODY_COUNT, AlignedVector<Eigen::Vector3d>(JointType_Count));
    body_joint_tracking_states_.resize(BODY_COUNT, std::vector<TrackingState>(JointType_Count));
}

int Kinect::bodyCount()
{
    return BODY_COUNT;
}

bool Kinect::initialize()
{
    return is_initialized_ || initializeDevice();
}

bool Kinect::initializeDevice()
{
    if (FAILED(GetDefaultKinectSensor(&sensor_)))
        return false;

    if (sensor_)
    {
        sensor_->get_CoordinateMapper(&mapper_);

        sensor_->Open();
        sensor_->OpenMultiSourceFrameReader(
                  FrameSourceTypes::FrameSourceTypes_Depth
                | FrameSourceTypes::FrameSourceTypes_Color
                | FrameSourceTypes::FrameSourceTypes_Body,
            &reader_);

        is_initialized_ = true;
        return reader_;
    }

    return false;
}

void Kinect::update()
{
    IMultiSourceFrame* frame;
    if (SUCCEEDED(reader_->AcquireLatestFrame(&frame)))
    {
        updateBodyData(frame);
        frame->Release();
    }
}

void Kinect::updateBodyData(IMultiSourceFrame* frame)
{
    IBodyFrameReference* body_frame_reference = 0;
    frame->get_BodyFrameReference(&body_frame_reference);

    IBodyFrame* body_frame = 0;
    body_frame_reference->AcquireFrame(&body_frame);

    if (body_frame)
        body_frame_reference->Release();

    if (!body_frame)
    {
        for (int i=0; i<BODY_COUNT; i++)
            is_body_tracked_[i] = false;
        return;
    }

    IBody* bodies[BODY_COUNT] = {0};
    body_frame->GetAndRefreshBodyData(BODY_COUNT, bodies);

    for (int i=0; i<BODY_COUNT; i++)
    {
        is_body_tracked_[i] = false;

        IBody* body = bodies[i];
        if (body)
        {
            BOOLEAN is_tracked;
            body->get_IsTracked(&is_tracked);
            
            if (is_tracked)
            {
                is_body_tracked_[i] = true;

                Joint joints[JointType_Count];
                body->GetJoints(JointType_Count, joints);

                for (int j=0; j<JointType_Count; j++)
                {
                    // joint positions
                    body_joint_positions_[i][ joints[j].JointType ] = Eigen::Vector3d(joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z);

                    // tracking state
                    switch (joints[j].TrackingState)
                    {
                    case ::TrackingState_Tracked:
                        body_joint_tracking_states_[i][ joints[j].JointType ] = TrackingState_Tracked;
                        break;
                    case ::TrackingState_Inferred:
                        body_joint_tracking_states_[i][ joints[j].JointType ] = TrackingState_Inferred;
                        break;
                    case ::TrackingState_NotTracked:
                        body_joint_tracking_states_[i][ joints[j].JointType ] = TrackingState_NotTracked;
                        break;
                    }
                }
            }
        }
    }

    // release
    for (int i=0; i<BODY_COUNT; i++)
    {
        if (bodies[i] != NULL)
            bodies[i]->Release();
    }

    body_frame->Release();
}

bool Kinect::isBodyTracked(int id)
{
    return is_body_tracked_[id];
}

const Eigen::Vector3d& Kinect::getBodyJointPosition(int body, JointType joint_type)
{
    return body_joint_positions_[body][joint_type];
}

}
