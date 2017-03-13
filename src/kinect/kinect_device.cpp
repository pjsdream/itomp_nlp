#include <itomp_nlp/kinect/kinect_device.h>

#include <Kinect.h>


namespace itomp
{

static const int width_ = 512;
static const int height_ = 424;
static const int colorwidth_ = 1920;
static const int colorheight_ = 1080;
static unsigned char rgb_image_[colorwidth_ * colorheight_ * 4]; // Stores RGB color image
static ColorSpacePoint depth_to_rgb_[width_ * height_];     // Maps depth pixels to rgb pixels
static CameraSpacePoint depth_to_xyz_[width_ * height_];    // Maps depth pixels to 3d coordinates

KinectDevice* KinectDevice::instance_ = 0;

KinectDevice* KinectDevice::getInstance()
{
    if (instance_ == 0)
        instance_ = new KinectDevice();

    return instance_;
}


KinectDevice::KinectDevice()
{
    is_body_tracked_.resize(BODY_COUNT, false);
    body_joint_positions_.resize(BODY_COUNT, AlignedVector<Eigen::Vector3d>(JointType_Count));
    body_joint_tracking_states_.resize(BODY_COUNT, std::vector<TrackingState>(JointType_Count));
}

int KinectDevice::bodyCount()
{
    return BODY_COUNT;
}

bool KinectDevice::initialize()
{
    return is_initialized_ || initializeDevice();
}

bool KinectDevice::initializeDevice()
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

void KinectDevice::update()
{
    IMultiSourceFrame* frame;
    if (SUCCEEDED(reader_->AcquireLatestFrame(&frame)))
    {
        updateBodyData(frame);
        updateDepthData(frame);
        updateColorData(frame);
        frame->Release();
    }
}

void KinectDevice::updateBodyData(IMultiSourceFrame* frame)
{
    IBodyFrameReference* body_frame_reference = 0;
    frame->get_BodyFrameReference(&body_frame_reference);

    IBodyFrame* body_frame = 0;
    body_frame_reference->AcquireFrame(&body_frame);

    if (body_frame)
        body_frame_reference->Release();

    if (!body_frame)
        return;

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

void KinectDevice::updateDepthData(IMultiSourceFrame* frame)
{
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) frameref->Release();

	if (!depthframe) return;

	// get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);

	// Write vertex coordinates
	mapper_->MapDepthFrameToCameraSpace(width_ * height_, buf, width_ * height_, depth_to_xyz_);
    depth_buffer_size_ = sz;

	// Fill in depth2rgb map
	mapper_->MapDepthFrameToColorSpace(width_ * height_, buf, width_ * height_, depth_to_rgb_);
	if (depthframe) depthframe->Release();
}

void KinectDevice::updateColorData(IMultiSourceFrame* frame)
{
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) frameref->Release();

	if (!colorframe) return;

	// Get data from frame
	colorframe->CopyConvertedFrameDataToArray(colorwidth_ * colorheight_ * 4, rgb_image_, ColorImageFormat_Rgba);

	if (colorframe) colorframe->Release();
}

unsigned int KinectDevice::getMaxNumPointCloud()
{
    return width_ * height_;
}

unsigned int KinectDevice::getNumPointCloud()
{
    return depth_buffer_size_;
}

void KinectDevice::getGLPointCloudDepths(GLubyte* depth)
{
	float* fdepth = (float*)depth;
	for (int i = 0; i < depth_buffer_size_; i++)
    {
		*fdepth++ = depth_to_xyz_[i].X;
		*fdepth++ = depth_to_xyz_[i].Y;
		*fdepth++ = depth_to_xyz_[i].Z;
	}
}

void KinectDevice::getGLPointCloudColors(GLubyte* color)
{
	// Write color array for vertices
	float* fcolor = (float*)color;
	for (int i = 0; i < width_ * height_; i++) {
		ColorSpacePoint p = depth_to_rgb_[i];
		// Check if color pixel coordinates are in bounds
		if (p.X < 0 || p.Y < 0 || p.X > colorwidth_ || p.Y > colorheight_) {
			*fcolor++ = 0;
			*fcolor++ = 0;
			*fcolor++ = 0;
		}
		else {
			int idx = (int)p.X + colorwidth_ * (int)p.Y;
			*fcolor++ = rgb_image_[4*idx + 0]/255.;
			*fcolor++ = rgb_image_[4*idx + 1]/255.;
			*fcolor++ = rgb_image_[4*idx + 2]/255.;
		}
		// Don't copy alpha channel
	}
}

bool KinectDevice::isBodyTracked(int id)
{
    return is_body_tracked_[id];
}

const Eigen::Vector3d& KinectDevice::getBodyJointPosition(int body, JointType joint_type)
{
    return body_joint_positions_[body][joint_type];
}

}
